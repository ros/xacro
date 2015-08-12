# Copyright (c) 2015, Open Source Robotics Foundation, Inc.
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Open Source Robotics Foundation, Inc.
#       nor the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior
#       written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Authors: Stuart Glaser, William Woodall, Robert Haschke
# Maintainer: Morgan Quigley <morgan@osrfoundation.org>

from __future__ import print_function, division

import glob
import os
import re
import sys
import ast
import math

from roslaunch import substitution_args
from rospkg.common import ResourceNotFound
from copy import deepcopy
from .color import warning, error, message
from .xmlutils import *
from .cli import process_args


try:
    _basestr = basestring
except NameError:
    _basestr = str

# Dictionary of substitution args
substitution_args_context = {}


# Stack of currently processed files
filestack = []

def push_file(filename):
    """
    Push a new filename to the filestack.
    Instead of directly modifying filestack, a deep-copy is created and modified,
    while the old filestack is returned.
    This allows to store the filestack that was active when a macro or property is defined
    """
    global filestack
    oldstack = filestack
    filestack = deepcopy(filestack)
    filestack.append(filename)
    return oldstack

def restore_filestack(oldstack):
    global filestack
    filestack = oldstack


def abs_filename_spec(filename_spec):
    """
    Prepend the dirname of the currently processed file
    if filename_spec is not yet absolute
    """
    if not os.path.isabs(filename_spec):
        parent_filename = filestack[-1]
        basedir = os.path.dirname(parent_filename) if parent_filename else '.'
        return os.path.join(basedir, filename_spec)
    return filename_spec


def load_yaml(filename):
    try:
        import yaml
    except:
        raise XacroException("yaml support not available; install python-yaml")

    filename = abs_filename_spec(filename)
    f = open(filename)
    oldstack = push_file(filename)
    try:
        return yaml.load(f)
    finally:
        f.close()
        restore_filestack(oldstack)


# global symbols dictionary
# taking simple security measures to forbid access to __builtins__
# only the very few symbols explicitly listed are allowed
# for discussion, see: http://nedbatchelder.com/blog/201206/eval_really_is_dangerous.html
global_symbols = {'__builtins__': {k: __builtins__[k] for k in ['list', 'dict', 'map', 'str', 'float', 'int']}}
# also define all math symbols and functions
global_symbols.update(math.__dict__)
# allow to import dicts from yaml
global_symbols.update(dict(load_yaml=load_yaml))


class XacroException(Exception):
    """
    XacroException allows to wrap another exception (exc) and to augment
    its error message: prefixing with msg and suffixing with suffix.
    str(e) finally prints: msg str(exc) suffix
    """
    def __init__(self, msg=None, suffix=None, exc=None, macro=None):
        super(XacroException, self).__init__(msg)
        self.suffix = suffix
        self.exc = exc
        self.macros = [] if macro is None else [macro]

    def __str__(self):
        return ' '.join([str(item) for item in
                         [self.message, self.exc, self.suffix] if item])


verbosity = 1
# deprecate non-namespaced use of xacro tags (issues #41, #59, #60)
def deprecated_tag(_issued=[False]):
    if _issued[0]:
        return
    _issued[0] = True

    if verbosity > 0:
        warning("deprecated: xacro tags should be prepended with 'xacro' xml namespace.")
        message("""Use the following script to fix incorrect usage:
        find . -iname "*.xacro" | xargs sed -i 's#<\([/]\\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\\1xacro:\\2#g'""")
        print_location(filestack)
        print(file=sys.stderr)


# require xacro namespace?
allow_non_prefixed_tags = True


def check_deprecated_tag(tag_name):
    """
    Check whether tagName starts with xacro prefix. If not, issue a warning.
    :param tag_name:
    :return: True if tagName is accepted as xacro tag
             False if tagName doesn't start with xacro prefix, but the prefix is required
    """
    if tag_name.startswith('xacro:'):
        return True
    else:
        if allow_non_prefixed_tags:
            deprecated_tag()
        return allow_non_prefixed_tags


class Macro(object):
    def __init__(self):
        self.body = None  # original xml.dom.Node
        self.params = []  # parsed parameter names
        self.defaultmap = {}  # default parameter values
        self.history = []  # definition history


def eval_extension(s):
    if s == '$(cwd)':
        return os.getcwd()
    try:
        return substitution_args.resolve_args(s, context=substitution_args_context, resolve_anon=False)
    except substitution_args.ArgException as e:
        raise XacroException("Undefined substitution argument", exc=e)
    except ResourceNotFound as e:
        raise XacroException("resource not found:", exc=e)


do_check_order=False
class Table(object):
    def __init__(self, parent=None):
        self.parent = parent
        self.table = {}
        self.unevaluated = set()  # set of unevaluated variables
        self.recursive = []  # list of currently resolved vars (to resolve recursive definitions)
        # the following variables are for debugging / checking only
        self.depth = self.parent.depth + 1 if self.parent else 0
        if do_check_order:
            # this is for smooth transition from deprecated to --inorder processing
            self.used = set() # set of used properties
            self.redefined = dict() # set of properties redefined after usage

    @staticmethod
    def _eval_literal(value):
        if isinstance(value, _basestr):
            try:
                # try to evaluate as literal, e.g. number, boolean, etc.
                # this is needed to handle numbers in property definitions as numbers, not strings
                evaluated = ast.literal_eval(value.strip())
                # However, (simple) list, tuple, dict expressions will be evaluated as such too,
                # which would break expected behaviour. Thus we only accept the evaluation otherwise.
                if not isinstance(evaluated, (list, dict, tuple)):
                    return evaluated
            except:
                pass

        return value

    def _resolve_(self, key):
        # lazy evaluation
        if key in self.unevaluated:
            if key in self.recursive:
                raise XacroException("recursive variable definition: %s" %
                                     " -> ".join(self.recursive + [key]))
            self.recursive.append(key)
            self.table[key] = self._eval_literal(eval_text(self.table[key], self))
            self.unevaluated.remove(key)
            self.recursive.remove(key)

        # return evaluated result
        value = self.table[key]
        if (verbosity > 2 and self.parent is None) or verbosity > 3:
            print("{indent}use {key}: {value} ({loc})".format(
                indent=self.depth*' ', key=key, value=value, loc=filestack[-1]), file=sys.stderr)
        if do_check_order:
            self.used.add(key)
        return value

    def __getitem__(self, key):
        if key in self.table:
            return self._resolve_(key)
        elif self.parent:
            return self.parent[key]
        else:
            raise KeyError(key)

    def _setitem(self, key, value, unevaluated):
        if do_check_order and key in self.used and key not in self.redefined:
            self.redefined[key] = filestack[-1]

        if key in global_symbols:
            warning("redefining global property: %s" % key)
            print_location(filestack)

        value = self._eval_literal(value)
        self.table[key] = value
        if unevaluated and isinstance(value, _basestr):
            # literal evaluation failed: re-evaluate lazily at first access
            self.unevaluated.add(key)
        elif key in self.unevaluated:
            # all other types cannot be evaluated
            self.unevaluated.remove(key)
        if (verbosity > 2 and self.parent is None) or verbosity > 3:
            print("{indent}set {key}: {value} ({loc})".format(
                indent=self.depth*' ', key=key, value=value, loc=filestack[-1]), file=sys.stderr)

    def __setitem__(self, key, value):
        self._setitem(key, value, unevaluated=True)

    def __contains__(self, key):
        return \
            key in self.table or \
            (self.parent and key in self.parent)

    def __str__(self):
        s = str(self.table)
        if isinstance(self.parent, Table):
            s += "\n  parent: "
            s += str(self.parent)
        return s

    def root(self):
        p = self
        while p.parent:
            p = p.parent
        return p

class NameSpace(object):
    # dot access (namespace.property) is forwarded to getitem()
    def __getattr__(self, item):
        return self.__getitem__(item)

class PropertyNameSpace(Table, NameSpace):
    def __init__(self, parent=None):
        super(PropertyNameSpace, self).__init__(parent)

class MacroNameSpace(dict, NameSpace):
    def __init__(self, *args, **kwargs):
        super(MacroNameSpace, self).__init__(*args, **kwargs)


class QuickLexer(object):
    def __init__(self, *args, **kwargs):
        if args:
            # copy attributes + variables from other instance
            other = args[0]
            self.__dict__.update(other.__dict__)
        else:
            self.res = []
            for k, v in kwargs.items():
                self.__setattr__(k, len(self.res))
                self.res.append(re.compile(v))
        self.str = ""
        self.top = None

    def lex(self, str):
        self.str = str
        self.top = None
        self.next()

    def peek(self):
        return self.top

    def next(self):
        result = self.top
        self.top = None
        for i in range(len(self.res)):
            m = self.res[i].match(self.str)
            if m:
                self.top = (i, m.group(0))
                self.str = self.str[m.end():]
                break
        return result


all_includes = []

include_no_matches_msg = """Include tag's filename spec \"{}\" matched no files."""


def is_include(elt):
    # Xacro should not use plain 'include' tags but only namespaced ones. Causes conflicts with
    # other XML elements including Gazebo's <gazebo> extensions
    if elt.tagName not in ['xacro:include', 'include']:
        return False

    # Temporary fix for ROS Hydro and the xacro include scope problem
    if elt.tagName == 'include':
        # check if there is any element within the <include> tag. mostly we are concerned
        # with Gazebo's <uri> element, but it could be anything. also, make sure the child
        # nodes aren't just a single Text node, which is still considered a deprecated
        # instance
        if elt.childNodes and not (len(elt.childNodes) == 1 and
                                   elt.childNodes[0].nodeType == elt.TEXT_NODE):
            # this is not intended to be a xacro element, so we can ignore it
            return False
        else:
            # throw a deprecated warning
            deprecated_tag()
    return True


def get_include_files(filename_spec, symbols):
    try:
        filename_spec = abs_filename_spec(eval_text(filename_spec, symbols))
    except XacroException as e:
        if e.exc and isinstance(e.exc, NameError) and symbols is None:
            raise XacroException('variable filename is supported with --inorder option only')
        else:
            raise

    if re.search('[*[?]+', filename_spec):
        # Globbing behaviour
        filenames = sorted(glob.glob(filename_spec))
        if len(filenames) == 0:
            warning(include_no_matches_msg.format(filename_spec))
    else:
        # Default behaviour
        filenames = [filename_spec]

    for filename in filenames:
        global all_includes
        all_includes.append(filename)
        yield filename


def import_xml_namespaces(parent, attributes):
    """import all namespace declarations into parent"""
    for name, value in attributes.items():
        if name.startswith('xmlns:'):
            oldAttr = parent.getAttributeNode(name)
            if oldAttr and oldAttr.value != value:
                warning("inconsistent namespace redefinitions for {name}:"
                        "\n old: {old}\n new: {new} ({new_file})".format(
                    name=name, old=oldAttr.value, new=value,
                    new_file=filestack[-1]))
            else:
                parent.setAttribute(name, value)


def process_include(elt, macros, symbols, func):
    included = []
    filename_spec, namespace_spec = check_attrs(elt, ['filename'], ['ns'])
    if namespace_spec:
        try:
            namespace_spec = eval_text(namespace_spec, symbols)
            macros[namespace_spec] = ns_macros = MacroNameSpace()
            symbols[namespace_spec] = ns_symbols = PropertyNameSpace()
            macros = ns_macros
            symbols = ns_symbols
        except TypeError:
            raise XacroException('namespaces are supported with --inorder option only')

    for filename in get_include_files(filename_spec, symbols):
        # extend filestack
        oldstack = push_file(filename)
        include = parse(None, filename).documentElement

        # recursive call to func
        func(include, macros, symbols)
        included.append(include)
        import_xml_namespaces(elt.parentNode, include.attributes)

        # restore filestack
        restore_filestack(oldstack)

    remove_previous_comments(elt)
    # replace the include tag with the nodes of the included file(s)
    replace_node(elt, by=included, content_only=True)


# @throws XacroException if a parsing error occurs with an included document
def process_includes(elt, macros=None, symbols=None):
    elt = first_child_element(elt)
    while elt:
        next = next_sibling_element(elt)
        if is_include(elt):
            process_include(elt, macros, symbols, process_includes)
        else:
            process_includes(elt)

        elt = next


def is_valid_name(name):
    """
    Checks whether name is a valid property or macro identifier.
    With python-based evaluation, we need to avoid name clashes with python keywords.
    """
    # Resulting AST of simple identifier is <Module [<Expr <Name "foo">>]>
    try:
        root = ast.parse(name)

        if isinstance(root, ast.Module) and \
           len(root.body) == 1 and isinstance(root.body[0], ast.Expr) and \
           isinstance(root.body[0].value, ast.Name) and root.body[0].value.id == name:
            return True
    except SyntaxError:
        pass

    return False


re_macro_arg = re.compile(r'''\s*([^\s:=]+?):?=(\^\|?)?((?:(?:'[^']*')?[^\s'"]*?)*)(?:\s+|$)(.*)''')
#                           space   param    :=   ^|   <--      default      -->   space    rest
def parse_macro_arg(s):
    """
    parse the first param spec from a macro parameter string s
    accepting the following syntax: <param>[:=|=][^|]<default>
    :param s: param spec string
    :return: param, (forward, default), rest-of-string
             forward will be either param or None (depending on whether ^ was specified)
             default will be the default string or None
             If there is no default spec at all, the middle pair will be replaced by None
    """
    m = re_macro_arg.match(s)
    if m:
        # there is a default value specified for param
        param, forward, default, rest = m.groups()
        if not default: default = None
        return param, (param if forward else None, default), rest
    else:
        # there is no default specified at all
        result = s.lstrip(' ').split(' ', 1)
        return result[0], None, result[1] if len(result) > 1 else ''


def grab_macro(elt, macros):
    assert(elt.tagName in ['macro', 'xacro:macro'])
    remove_previous_comments(elt)

    name, params = check_attrs(elt, ['name'], ['params'])
    if name == 'call':
        warning("deprecated use of macro name 'call'; xacro:call became a new keyword")
    if name.find('.') != -1:
        warning("macro names must not contain '.': %s" % name)

    # fetch existing or create new macro definition
    macro = macros.get(name, Macro())
    # append current filestack to history
    macro.history.append(filestack)
    macro.body = elt

    # parse params and their defaults
    macro.params = []
    macro.defaultmap = {}
    while params:
        param, value, params = parse_macro_arg(params)
        macro.params.append(param)
        if value is not None:
            macro.defaultmap[param] = value  # parameter with default

    macros[name] = macro
    replace_node(elt, by=None)


# Fill the dictionary { macro_name => macro_xml_block }
def grab_macros(elt, macros):
    elt = first_child_element(elt)
    while elt:
        next = next_sibling_element(elt)
        if elt.tagName in ['macro', 'xacro:macro'] \
                and check_deprecated_tag(elt.tagName):
            grab_macro(elt, macros)
        else:
            grab_macros(elt, macros)

        elt = next


def grab_property(elt, table):
    assert(elt.tagName in ['property', 'xacro:property'])
    remove_previous_comments(elt)

    name, value, scope = check_attrs(elt, ['name'], ['value', 'scope'])
    if not is_valid_name(name):
        raise XacroException('Property names must be valid python identifiers: ' + name)

    if value is None:
        name = '**' + name
        value = elt  # debug

    if scope and scope == 'global':
        target_table = table.root()
        unevaluated = False
    elif scope and scope == 'parent':
        if table.parent:
            target_table = table.parent
        else:
            warning("%s: no parent scope at global scope " % name)
        unevaluated = False
    else:
        target_table = table
        unevaluated = True

    if not unevaluated and isinstance(value, _basestr):
        value = eval_text(value, table)

    target_table._setitem(name, value, unevaluated=unevaluated)
    replace_node(elt, by=None)


# Fill the table of the properties
def grab_properties(elt, table):
    elt = first_child_element(elt)
    while elt:
        next = next_sibling_element(elt)
        if elt.tagName in ['property', 'xacro:property'] \
                and check_deprecated_tag(elt.tagName):
            grab_property(elt, table)
        else:
            grab_properties(elt, table)

        elt = next


LEXER = QuickLexer(DOLLAR_DOLLAR_BRACE=r"\$\$+\{",
                   EXPR=r"\$\{[^\}]*\}",
                   EXTENSION=r"\$\([^\)]*\)",
                   TEXT=r"([^\$]|\$[^{(]|\$$)+")


# evaluate text and return typed value
def eval_text(text, symbols):
    def handle_expr(s):
        try:
            return eval(s, global_symbols, symbols)
        except Exception as e:
            # re-raise as XacroException to add more context
            raise XacroException(exc=e,
                suffix=os.linesep + "when evaluating expression '%s'" % s)

    def handle_extension(s):
        return eval_extension("$(%s)" % s)

    results = []
    lex = QuickLexer(LEXER)
    lex.lex(text)
    while lex.peek():
        if lex.peek()[0] == lex.EXPR:
            results.append(handle_expr(lex.next()[1][2:-1]))
        elif lex.peek()[0] == lex.EXTENSION:
            results.append(handle_extension(lex.next()[1][2:-1]))
        elif lex.peek()[0] == lex.TEXT:
            results.append(lex.next()[1])
        elif lex.peek()[0] == lex.DOLLAR_DOLLAR_BRACE:
            results.append(lex.next()[1][1:])
    # return single element as is, i.e. typed
    if len(results) == 1:
        return results[0]
    # otherwise join elements to a string
    else:
        return ''.join(map(str, results))


def eval_default_arg(forward_variable, default, symbols, macro):
    if forward_variable is None:
        return eval_text(default, symbols)
    try:
        return symbols[forward_variable]
    except KeyError:
        if default is not None:
            return eval_text(default, symbols)
        else:
            raise XacroException("Undefined property to forward: " + forward_variable, macro=macro)


def handle_dynamic_macro_call(node, macros, symbols):
    name, = reqd_attrs(node, ['macro'])
    if not name:
        raise XacroException("xacro:call is missing the 'macro' attribute")
    name = str(eval_text(name, symbols))

    # remove 'macro' attribute and rename tag with resolved macro name
    node.removeAttribute('macro')
    node.tagName = name
    # forward to handle_macro_call
    result = handle_macro_call(node, macros, symbols)
    if not result:  # we expect the call to succeed
        raise XacroException("unknown macro name '%s' in xacro:call" % name)
    return True

def resolve_macro(fullname, macros):
    # split name into namespaces and real name
    namespaces = fullname.split('.')
    name = namespaces.pop(-1)
    # move xacro: prefix from first namespace to name
    if namespaces and namespaces[0].startswith('xacro:'):
        namespaces[0] = namespaces[0].replace('xacro:', '')
        name = 'xacro:' + name

    def _resolve(namespaces, name, macros):
        # traverse namespaces to actual macros dict
        for ns in namespaces:
            macros = macros[ns]
        try:
            return macros[name]
        except KeyError:
            # try without xacro: prefix as well
            if name.startswith('xacro:'):
                return _resolve([], name.replace('xacro:',''), macros)

    # try fullname and (namespaces, name) in this order
    m = _resolve([], fullname, macros)
    if m: return m
    elif namespaces: return _resolve(namespaces, name, macros)\


def handle_macro_call(node, macros, symbols):
    try:
        m = resolve_macro(node.tagName, macros)
        body = m.body.cloneNode(deep=True)

    except (KeyError, TypeError, AttributeError):
        # TODO If deprecation runs out, this test should be moved up front
        if node.tagName == 'xacro:call':
            return handle_dynamic_macro_call(node, macros, symbols)
        return False  # no macro

    # Expand the macro
    scoped = Table(symbols)  # new local name space for macro evaluation
    params = m.params[:]  # deep copy macro's params list
    for name, value in node.attributes.items():
        if name not in params:
            raise XacroException("Invalid parameter \"%s\"" % str(name), macro=m)
        params.remove(name)
        scoped._setitem(name, eval_text(value, symbols), unevaluated=False)
        node.setAttribute(name, "")  # suppress second evaluation in eval_all()

    # Evaluate block parameters in node
    eval_all(node, macros, symbols)

    # Fetch block parameters, in order
    block = first_child_element(node)
    for param in params[:]:
        if param[0] == '*':
            if not block:
                raise XacroException("Not enough blocks", macro=m)
            params.remove(param)
            scoped[param] = block
            block = next_sibling_element(block)

    if block is not None:
        raise XacroException("Unused block \"%s\"" % block.tagName, macro=m)

    # Try to load defaults for any remaining non-block parameters
    for param in params[:]:
        # block parameters are not supported for defaults
        if param[0] == '*': continue

        # get default
        name, default = m.defaultmap.get(param, (None,None))
        if name is not None or default is not None:
            scoped._setitem(param, eval_default_arg(name, default, symbols, m), unevaluated=False)
            params.remove(param)

    if params:
        raise XacroException("Undefined parameters [%s]" % ",".join(params), macro=m)

    try:
        eval_all(body, macros, scoped)
    except Exception as e:
        if hasattr(e, 'macros'):
            e.macros.append(m)
        else:
            e.macros = [m]
        raise

    # Replaces the macro node with the expansion
    remove_previous_comments(node)
    replace_node(node, by=body, content_only=True)
    return True


def get_boolean_value(value, condition):
    """
    Return a boolean value that corresponds to the given Xacro condition value.
    Values "true", "1" and "1.0" are supposed to be True.
    Values "false", "0" and "0.0" are supposed to be False.
    All other values raise an exception.

    :param value: The value to be evaluated. The value has to already be evaluated by Xacro.
    :param condition: The original condition text in the XML.
    :return: The corresponding boolean value, or a Python expression that, converted to boolean, corresponds to it.
    :raises ValueError: If the condition value is incorrect.
    """
    try:
        if isinstance(value, _basestr):
            if value == 'true': return True
            elif value == 'false': return False
            else: return ast.literal_eval(value)
        else:
            return bool(value)
    except:
        raise XacroException("Xacro conditional \"%s\" evaluated to \"%s\", "
                             "which is not a boolean expression." % (condition, value))


_empty_text_node = xml.dom.minidom.getDOMImplementation().createDocument(None, "dummy", None).createTextNode('\n\n')
def remove_previous_comments(node):
    """remove consecutive comments in front of the xacro-specific node"""
    next = node.nextSibling
    previous = node.previousSibling
    while previous:
        if previous.nodeType == xml.dom.Node.TEXT_NODE and \
                previous.data.isspace() and previous.data.count('\n') <= 1:
            previous = previous.previousSibling  # skip a single empty text node (max 1 newline)

        if previous and previous.nodeType == xml.dom.Node.COMMENT_NODE:
            comment = previous
            previous = previous.previousSibling
            node.parentNode.removeChild(comment)
        else:
            # insert empty text node to stop removing of comments in future calls
            # actually this moves the singleton instance to the new location
            if next: node.parentNode.insertBefore(_empty_text_node, next)
            return


def eval_all(node, macros, symbols):
    """Recursively evaluate node, expanding macros, replacing properties, and evaluating expressions"""
    # evaluate the attributes
    for name, value in node.attributes.items():
        result = str(eval_text(value, symbols))
        node.setAttribute(name, result)

    node = node.firstChild
    while node:
        next = node.nextSibling
        if node.nodeType == xml.dom.Node.ELEMENT_NODE:
            if node.tagName in ['insert_block', 'xacro:insert_block'] \
                    and check_deprecated_tag(node.tagName):
                name, = check_attrs(node, ['name'], [])

                if ("**" + name) in symbols:
                    # Multi-block
                    block = symbols['**' + name]
                    content_only = True
                elif ("*" + name) in symbols:
                    # Single block
                    block = symbols['*' + name]
                    content_only = False
                else:
                    raise XacroException("Undefined block \"%s\"" % name)

                # cloning block allows to insert the same block multiple times
                block = block.cloneNode(deep=True)
                # recursively evaluate block
                eval_all(block, macros, symbols)
                replace_node(node, by=block, content_only=content_only)

            elif is_include(node):
                process_include(node, macros, symbols, eval_all)

            elif node.tagName in ['property', 'xacro:property'] \
                    and check_deprecated_tag(node.tagName):
                grab_property(node, symbols)

            elif node.tagName in ['macro', 'xacro:macro'] \
                    and check_deprecated_tag(node.tagName):
                grab_macro(node, macros)

            elif node.tagName in ['arg', 'xacro:arg'] \
                    and check_deprecated_tag(node.tagName):
                name, default = check_attrs(node, ['name', 'default'], [])
                if name not in substitution_args_context['arg']:
                    substitution_args_context['arg'][name] = eval_text(default, symbols)

                remove_previous_comments(node)
                replace_node(node, by=None)

            elif node.tagName == 'xacro:element':
                name = eval_text(*reqd_attrs(node, ['xacro:name']), symbols=symbols)
                if not name:
                    raise XacroException("xacro:element: empty name")

                node.removeAttribute('xacro:name')
                node.nodeName = node.tagName = name
                continue  # re-process the node with new tagName

            elif node.tagName == 'xacro:attribute':
                name, value = [eval_text(a, symbols) for a in reqd_attrs(node, ['name', 'value'])]
                if not name:
                    raise XacroException("xacro:attribute: empty name")

                node.parentNode.setAttribute(name, value)
                replace_node(node, by=None)

            elif node.tagName in ['if', 'xacro:if', 'unless', 'xacro:unless'] \
                    and check_deprecated_tag(node.tagName):
                remove_previous_comments(node)
                cond, = check_attrs(node, ['value'], [])
                keep = get_boolean_value(eval_text(cond, symbols), cond)
                if node.tagName in ['unless', 'xacro:unless']:
                    keep = not keep

                if keep:
                    eval_all(node, macros, symbols)
                    replace_node(node, by=node, content_only=True)
                else:
                    replace_node(node, by=None)

            elif handle_macro_call(node, macros, symbols):
                pass  # handle_macro_call does all the work of expanding the macro

            else:
                # these are the non-xacro tags
                if node.tagName.startswith("xacro:"):
                    raise XacroException("unknown macro name: %s" % node.tagName)

                eval_all(node, macros, symbols)

        # TODO: Also evaluate content of COMMENT_NODEs?
        elif node.nodeType == xml.dom.Node.TEXT_NODE:
            node.data = str(eval_text(node.data, symbols))

        node = next


def parse(inp, filename=None):
    """
    Parse input or filename into a DOM tree.
    If inp is None, open filename and load from there.
    Otherwise, parse inp, either as string or file object.
    If inp is already a DOM tree, this function is a noop.
    :return:xml.dom.minidom.Document
    :raise: xml.parsers.expat.ExpatError
    """
    f = None
    if inp is None:
        try:
            inp = f = open(filename)
        except IOError as e:
            # do not report currently processed file as "in file ..."
            filestack.pop()
            raise XacroException(e.strerror + ": " + e.filename)

    try:
        if isinstance(inp, _basestr):
            return xml.dom.minidom.parseString(inp)
        elif isinstance(inp, file):
            return xml.dom.minidom.parse(inp)
        return inp

    finally:
        if f:
            f.close()


def process_doc(doc,
                in_order=False, just_deps=False, just_includes=False,
                mappings=None, xacro_ns=True, **kwargs):
    global verbosity, do_check_order
    verbosity = kwargs.get('verbosity', verbosity)
    do_check_order = kwargs.get('do_check_order', do_check_order)

    # set substitution args
    if mappings is not None:
        substitution_args_context['arg'] = mappings

    global allow_non_prefixed_tags
    allow_non_prefixed_tags = xacro_ns

    # if not yet defined: initialize filestack
    if not filestack: restore_filestack([None])

    # inorder processing requires to process the whole document for deps too
    # because filenames might be specified via properties or macro parameters
    if (just_deps or just_includes) and not in_order:
        process_includes(doc.documentElement)
        return

    macros = {}
    symbols = Table()
    if not in_order:
        # process includes, macros, and properties before evaluating stuff
        process_includes(doc.documentElement)
        grab_macros(doc, macros)
        grab_properties(doc, symbols)

    eval_all(doc.documentElement, macros, symbols)

    # reset substitution args
    substitution_args_context['arg'] = {}

    if do_check_order and symbols.redefined:
        warning("Document is incompatible to --inorder processing.")
        warning("The following properties were redefined after usage:")
        for k, v in symbols.redefined.iteritems():
            message(k, "redefined in", v, color='yellow')


def open_output(output_filename):
    if output_filename is None:
        return sys.stdout
    else:
        dir_name = os.path.dirname(output_filename)
        if dir_name:
            try:
                os.makedirs(dir_name)
            except os.error:
                # errors occur when dir_name exists or creation failed
                # ignore error here; opening of file will fail if directory is still missing
                pass

        try:
            return open(output_filename, 'w')
        except IOError as e:
            raise XacroException("Failed to open output:", exc=e)


def print_location(filestack, err=None, file=sys.stderr):
    macros = getattr(err, 'macros', []) if err else []
    msg = 'when instantiating macro:'
    for m in macros:
        name = m.body.getAttribute('name')
        location = '(%s)' % m.history[-1][-1]
        print(msg, name, location, file=file)
        msg = 'instantiated from:'

    msg = 'in file:' if macros else 'when processing file:'
    for f in reversed(filestack):
        if f is None: f = 'string'
        print(msg, f, file=file)
        msg = 'included from:'


def main():
    opts, input_file = process_args(sys.argv[1:])
    if opts.in_order == False:
        warning("Traditional processing is deprecated. Switch to --inorder processing!")
        message("To check for compatibility of your document, use option --check-order.", color='yellow')
        message("For more infos, see http://wiki.ros.org/xacro#Processing_Order", color='yellow')

    try:
        restore_filestack([input_file])
        doc = parse(None, input_file)
        process_doc(doc, **vars(opts))
        out = open_output(opts.output)

    except xml.parsers.expat.ExpatError as e:
        error("XML parsing error: %s" % str(e), alt_text=None)
        if verbosity > 0:
            print_location(filestack, e)
            print(file=sys.stderr) # add empty separator line before error
            print("Check that:", file=sys.stderr)
            print(" - Your XML is well-formed", file=sys.stderr)
            print(" - You have the xacro xmlns declaration:",
                  "xmlns:xacro=\"http://www.ros.org/wiki/xacro\"", file=sys.stderr)
        sys.exit(2)  # indicate failure, but don't print stack trace on XML errors

    except Exception as e:
        error(str(e))
        if verbosity > 0:
            print_location(filestack, e)
        if verbosity > 1:
            print(file=sys.stderr)  # add empty separator line before error
            raise  # create stack trace
        else:
            sys.exit(2)  # gracefully exit with error condition

    if opts.just_deps:
        out.write(" ".join(set(all_includes)))
        print()
        return
    if opts.just_includes:
        out.write(doc.toprettyxml(indent='  '))
        print()
        return

    banner = [xml.dom.minidom.Comment(c) for c in
              [" %s " % ('=' * 83),
               " |    This document was autogenerated by xacro from %-30s | " % input_file,
               " |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED  %-30s | " % "",
               " %s " % ('=' * 83)]]
    first = doc.firstChild
    for comment in banner:
        doc.insertBefore(comment, first)

    out.write(doc.toprettyxml(indent='  '))
    print()
    if opts.output:
        out.close()
