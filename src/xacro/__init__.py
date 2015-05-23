# Copyright (c) 2013, Willow Garage, Inc.
# Copyright (c) 2014, Open Source Robotics Foundation, Inc.
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

# Author: Stuart Glaser
# Maintainer: William Woodall <william@osrfoundation.org>

from __future__ import print_function, division

import glob
import os
import re
import string
import sys
import xml
import ast
import math

import rospy
from roslaunch import substitution_args
from rosgraph.names import load_mappings
from optparse import OptionParser

try:
    _basestr = basestring
except NameError:
    _basestr = str

# Dictionary of substitution args
substitution_args_context = {}

# global symbols dictionary
# taking simple security measures to forbid access to __builtins__
# only the very few symbols explicitly listed are allowed
# for discussion, see: http://nedbatchelder.com/blog/201206/eval_really_is_dangerous.html
global_symbols = {'__builtins__':{k: __builtins__[k] for k in ['list', 'dict', 'map', 'str', 'float', 'int']}}
# also define all math symbols and functions
global_symbols.update(math.__dict__)

class XacroException(Exception):
    pass


def eval_extension(str):
    if str == '$(cwd)':
        return os.getcwd()
    return substitution_args.resolve_args(str, context=substitution_args_context, resolve_anon=False)


# Better pretty printing of xml
# Taken from http://ronrothman.com/public/leftbraned/xml-dom-minidom-toprettyxml-and-silly-whitespace/
def fixed_writexml(self, writer, indent="", addindent="", newl=""):
    # indent = current indentation
    # addindent = indentation to add to higher levels
    # newl = newline string
    writer.write(indent + "<" + self.tagName)

    attrs = self._get_attributes()
    a_names = list(attrs.keys())
    a_names.sort()

    for a_name in a_names:
        writer.write(" %s=\"" % a_name)
        xml.dom.minidom._write_data(writer, attrs[a_name].value)
        writer.write("\"")
    if self.childNodes:
        if len(self.childNodes) == 1 \
           and self.childNodes[0].nodeType == xml.dom.minidom.Node.TEXT_NODE:
            writer.write(">")
            self.childNodes[0].writexml(writer, "", "", "")
            writer.write("</%s>%s" % (self.tagName, newl))
            return
        writer.write(">%s" % (newl))
        for node in self.childNodes:
            # skip whitespace-only text nodes
            if node.nodeType == xml.dom.minidom.Node.TEXT_NODE and \
               not node.data.strip():
                continue
            node.writexml(writer, indent + addindent, addindent, newl)
        writer.write("%s</%s>%s" % (indent, self.tagName, newl))
    else:
        writer.write("/>%s" % (newl))
# replace minidom's function with ours
xml.dom.minidom.Element.writexml = fixed_writexml


class Table:
    def __init__(self, parent=None):
        self.parent = parent
        self.table = {}
        self.unevaluated = set() # set of unevaluated variables
        self.recursive = [] # list of currently resolved vars (to resolve recursive definitions)

    def _eval_literal(self, value):
        if isinstance(value, _basestr):
            try:
                # try to evaluate as literal, e.g. number, boolean, etc.
                # this is needed to handle numbers in property definitions as numbers, not strings
                evaluated = ast.literal_eval(value)
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
        return self.table[key]

    def __getitem__(self, key):
        if key in self.table:
            return self._resolve_(key)
        elif self.parent:
            return self.parent[key]
        else:
            raise KeyError(key)

    def __setitem__(self, key, value):
        value = self._eval_literal(value)
        self.table[key] = value
        if isinstance(value, _basestr):
            # strings need to be evaluated again at first access
            self.unevaluated.add(key)
        elif key in self.unevaluated:
            # all other types cannot be evaluated
            self.unevaluated.remove(key)

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


def first_child_element(elt):
    c = elt.firstChild
    while c:
        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
            return c
        c = c.nextSibling
    return None


def next_sibling_element(elt):
    c = elt.nextSibling
    while c:
        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
            return c
        c = c.nextSibling
    return None


# Pre-order traversal of the elements
def next_element(elt):
    child = first_child_element(elt)
    if child:
        return child
    while elt and elt.nodeType == xml.dom.Node.ELEMENT_NODE:
        next = next_sibling_element(elt)
        if next:
            return next
        elt = elt.parentNode
    return None


# Pre-order traversal of all the nodes
def next_node(node):
    if node.firstChild:
        return node.firstChild
    while node:
        if node.nextSibling:
            return node.nextSibling
        node = node.parentNode
    return None


def child_nodes(elt):
    c = elt.firstChild
    while c:
        yield c
        c = c.nextSibling

all_includes = []

# Deprecated message for <include> tags that don't have <xacro:include> prepended:
deprecated_include_msg = """DEPRECATED IN HYDRO:
  The <include> tag should be prepended with 'xacro' if that is the intended use
  of it, such as <xacro:include ...>. Use the following script to fix incorrect
  xacro includes:
     sed -i 's/<include/<xacro:include/g' `find . -iname *.xacro`"""

include_no_matches_msg = """Include tag's filename spec \"{}\" matched no files."""


def is_include(elt):
    # Xacro should not use plain 'include' tags but only namespaced ones. Causes conflicts with
    # other XML elements including Gazebo's <gazebo> extensions
    if elt.tagName not in ['xacro:include', 'include']: return False

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
            print(deprecated_include_msg, file=sys.stderr)
    return True


def parse_includes(elt, parent_filename, symbols):
    filename_spec = eval_text(elt.getAttribute('filename'), symbols)
    if not os.path.isabs(filename_spec):
        basedir = os.path.dirname(parent_filename) if parent_filename else '.'
        filename_spec = os.path.join(basedir, filename_spec)

    if re.search('[*[?]+', filename_spec):
        # Globbing behaviour
        filenames = sorted(glob.glob(filename_spec))
        if len(filenames) == 0:
            print(include_no_matches_msg.format(filename_spec), file=sys.stderr)
    else:
        # Default behaviour
        filenames = [filename_spec]

    for filename in filenames:
        global all_includes
        all_includes.append(filename)

        try:
            yield parse(None, filename), filename
        except IOError as e:
            raise XacroException("failed to open include file: %s\n%s" % (filename, str(e)))


def process_include(elt, included):
    # Replaces the include tag with the nodes of the included file
    for c in child_nodes(included.documentElement):
        elt.parentNode.insertBefore(c.cloneNode(deep=True), elt)

    # Makes sure the final document declares all the namespaces of the included documents.
    for name, value in included.documentElement.attributes.items():
        if name.startswith('xmlns:'):
            elt.parentNode.setAttribute(name, value)


def print_location_msg(e, filename):
    msg = 'error in file:' if getattr(e, '_xacro_first', True) else 'included from:'
    e._xacro_first = False
    if filename:
        print(msg, filename, file=sys.stderr)


## @throws XacroException if a parsing error occurs with an included document
def process_includes(doc, filename):
    previous = doc.documentElement
    elt = next_element(previous)
    while elt:
        if is_include(elt):
            try:
                for included, included_filename in parse_includes(elt, filename, {}):
                    # recursively process includes
                    process_includes(included, included_filename)
                    # embed included doc before elt

                    process_include(elt, included)
            except Exception as e:
                print_location_msg(e, filename)
                raise

            elt.parentNode.removeChild(elt)

        else:
            previous = elt

        elt = next_element(previous)

def grab_macro(elt, macros):
    if elt.tagName not in ['macro', 'xacro:macro']: 
        raise XacroException("expected macro element")

    name = elt.getAttribute('name')
    macros[name] = elt
    macros['xacro:' + name] = elt
    elt.parentNode.removeChild(elt)

# Returns a dictionary: { macro_name => macro_xml_block }
def grab_macros(doc):
    macros = {}

    previous = doc.documentElement
    elt = next_element(previous)
    while elt:
        if elt.tagName == 'macro' or elt.tagName == 'xacro:macro':
            grab_macro(elt, macros)
        else:
            previous = elt

        elt = next_element(previous)
    return macros

def grab_property(elt, table):
    if elt.tagName not in ['property', 'xacro:property']: 
        raise XacroException("expected property element")

    name = elt.getAttribute('name')
    value = None

    if elt.hasAttribute('value'):
        value = elt.getAttribute('value')
    else:
        name = '**' + name
        value = elt  # debug

    elt.parentNode.removeChild(elt)

    bad = string.whitespace + "${}"
    if any(ch in name for ch in bad):
        sys.stderr.write('Property names may not have whitespace, ' +
                         '"{", "}", or "$" : "' + name + '"')
        return

    table[name] = value

# Returns a Table of the properties
def grab_properties(doc, table=Table()):
    previous = doc.documentElement
    elt = next_element(previous)
    while elt:
        if elt.tagName == 'property' or elt.tagName == 'xacro:property':
            grab_property(elt, table)
        else:
            previous = elt

        elt = next_element(previous)
    return table

LEXER = QuickLexer(DOLLAR_DOLLAR_BRACE=r"\$\$+\{",
                   EXPR=r"\$\{[^\}]*\}",
                   EXTENSION=r"\$\([^\)]*\)",
                   TEXT=r"([^\$]|\$[^{(]|\$$)+")
# evaluate text and return typed value
def eval_text(text, symbols):
    def handle_expr(s):
        try:
            return eval(s, global_symbols, symbols)
        except NameError as e:
            raise XacroException("%s evaluating expression '%s'" % (str(e), s))
        except Exception as e:
            raise

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
    if len(results) == 1: return results[0]
    # otherwise join elements to a string
    else: return ''.join(map(str, results))

# Assuming a node xacro:call, this function resolves the final macro name and
# adapts the node's tagName accordingly to pretend a call for this macro
def handle_dynamic_macro_name(node, macros, symbols):
    if node.tagName in macros:
        print ("DEPRECATED use of macro name 'call'. xacro:call became a new keyword.", file=sys.stderr)
        return

    name = node.getAttribute('macro')
    if name == None:
        raise XacroException("xacro:call is missing the 'macro' attribute")

    name = str(eval_text(name, symbols))
    if name not in macros:
        raise XacroException("unknown macro name '%s' in xacro:call" % name)

    # finally remove 'macro' attribute and replace tagName for the resolved macro name
    node.removeAttribute('macro')
    node.tagName = name

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
            if   value == 'true': return True
            elif value == 'false': return False
            else: return ast.literal_eval(value)
        else: return bool(value)
    except:
        raise XacroException("Xacro conditional \"%s\" evaluated to \"%s\", which is not a boolean expression." % (condition, value))


# Expands macros, replaces properties, and evaluates expressions
def eval_all(root, filename, macros={}, symbols=Table()):
    # Evaluates the attributes for the root node
    for at in root.attributes.items():
        result = str(eval_text(at[1], symbols))
        root.setAttribute(at[0], result)

    previous = root
    node = next_node(previous)
    while node:
        if node.nodeType == xml.dom.Node.ELEMENT_NODE:
            if is_include(node):
                try:
                    for included, included_filename in parse_includes(node, filename, symbols):
                        # recursively process includes
                        eval_all(included.documentElement, included_filename, macros, symbols)
                        # embed included doc before node
                        process_include(node, included)
                except Exception as e:
                    print_location_msg(e, filename)
                    raise

                node.parentNode.removeChild(node)
                node = next_node(previous)
                continue

            if node.tagName in ['property', 'xacro:property']:
                grab_property(node, symbols)
                node = next_node(previous)
                continue

            if node.tagName in ['macro', 'xacro:macro']:
                grab_macro(node, macros)
                node = next_node(previous)
                continue

            if node.tagName in ['xacro:call']:
                handle_dynamic_macro_name(node, macros, symbols)

            if node.tagName in macros:
                body = macros[node.tagName].cloneNode(deep=True)
                params = body.getAttribute('params').split()

                # Parse default values for any parameters
                defaultmap = {}
                for param in params[:]:
                    splitParam = param.split(':=')

                    if len(splitParam) == 2:
                        defaultmap[splitParam[0]] = splitParam[1]
                        params.remove(param)
                        params.append(splitParam[0])

                    elif len(splitParam) != 1:
                        raise XacroException("Invalid parameter definition")

                # Expands the macro
                scoped = Table(symbols)
                for name, value in node.attributes.items():
                    if not name in params:
                        raise XacroException("Invalid parameter \"%s\" while expanding macro \"%s\"" %
                                             (str(name), str(node.tagName)))
                    params.remove(name)
                    scoped[name] = eval_text(value, symbols)

                # Pulls out the block arguments, in order
                cloned = node.cloneNode(deep=True)
                eval_all(cloned, filename, macros, symbols)
                block = cloned.firstChild
                for param in params[:]:
                    if param[0] == '*':
                        while block and block.nodeType != xml.dom.Node.ELEMENT_NODE:
                            block = block.nextSibling
                        if not block:
                            raise XacroException("Not enough blocks while evaluating macro %s" % str(node.tagName))
                        params.remove(param)
                        scoped[param] = block
                        block = block.nextSibling

                # Try to load defaults for any remaining non-block parameters
                for param in params[:]:
                    if param[0] != '*' and param in defaultmap:
                        scoped[param] = defaultmap[param]
                        params.remove(param)

                if params:
                    raise XacroException("Parameters [%s] were not set for macro %s" %
                                         (",".join(params), str(node.tagName)))
                eval_all(body, filename, macros, scoped)

                # Replaces the macro node with the expansion
                for e in list(child_nodes(body)):  # Ew
                    node.parentNode.insertBefore(e, node)
                node.parentNode.removeChild(node)

                node = None

            elif node.tagName == 'xacro:arg':
                name = node.getAttribute('name')
                if not name:
                    raise XacroException("Argument name missing")
                default = node.getAttribute('default')
                if default and name not in substitution_args_context['arg']:
                    substitution_args_context['arg'][name] = default

                node.parentNode.removeChild(node)
                node = None

            elif node.tagName == 'insert_block' or node.tagName == 'xacro:insert_block':
                name = node.getAttribute('name')

                if ("**" + name) in symbols:
                    # Multi-block
                    block = symbols['**' + name]

                    for e in list(child_nodes(block)):
                        node.parentNode.insertBefore(e.cloneNode(deep=True), node)
                    node.parentNode.removeChild(node)
                elif ("*" + name) in symbols:
                    # Single block
                    block = symbols['*' + name]

                    node.parentNode.insertBefore(block.cloneNode(deep=True), node)
                    node.parentNode.removeChild(node)
                else:
                    raise XacroException("Block \"%s\" was never declared" % name)

                node = None

            elif node.tagName in ['if', 'xacro:if', 'unless', 'xacro:unless']:
                cond = node.getAttribute('value')
                keep = get_boolean_value(eval_text(cond, symbols), cond)
                if node.tagName in ['unless', 'xacro:unless']: keep = not keep
                if keep:
                    for e in list(child_nodes(node)):
                        node.parentNode.insertBefore(e, node)

                node.parentNode.removeChild(node)

            else:
                if node.tagName.startswith("xacro:"):
                    raise XacroException("unknown macro name: %s" % node.tagName)

                # Evals the attributes
                for at in node.attributes.items():
                    result = str(eval_text(at[1], symbols))
                    node.setAttribute(at[0], result)
                previous = node

        elif node.nodeType == xml.dom.Node.TEXT_NODE:
            node.data = str(eval_text(node.data, symbols))
            previous = node
        else:
            previous = node

        node = next_node(previous)
    return macros


def process_cli_args(argv, require_input=True):
    args = {}
    parser = OptionParser(usage="usage: %prog [options] <input>")
    parser.add_option("-o", dest="output", metavar="FILE",
                      help="write output to FILE instead of stdout")
    parser.add_option("--inorder", action="store_true", dest="in_order",
                      help="evaluate document in read order")
    parser.add_option("--deps", action="store_true", dest="just_deps",
                      help="print file dependencies")
    parser.add_option("--includes", action="store_true", dest="just_includes",
                      help="only process includes")
    parser.add_option("--debug", action="store_true", dest="debug",
                      help="print stack trace on exceptions")

    # process substitution args
    mappings = load_mappings(argv)

    parser.set_defaults(in_order=False, just_deps=False, just_includes=False)
    (options, pos_args) = parser.parse_args(rospy.myargv(argv))

    if len(pos_args) != 1:
        if require_input:
            parser.error("expected exactly one input file as argument")
        else:
            pos_args = [None]

    options.mappings = mappings
    return options, pos_args[0]


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
        inp = f = open(filename)

    try:
        if isinstance(inp, _basestr):
            return xml.dom.minidom.parseString(inp)
        elif isinstance(inp, file):
            return xml.dom.minidom.parse(inp)
        return inp

    finally:
        if f: f.close()


def process_doc(doc, filename=None,
                in_order=False, just_deps=False, just_includes=False,
                mappings=None, **kwargs):
    # set substitution args
    if mappings is not None:
        substitution_args_context['arg'] = mappings

    if just_deps or just_includes:
        process_includes(doc, filename)
        return

    if not in_order:
        # process includes, macros, and properties before evaluating stuff
        process_includes(doc, filename)
        macros = grab_macros(doc)
        symbols = grab_properties(doc, Table())
    else:
        macros  = {}
        symbols = Table()

    eval_all(doc.documentElement, filename, macros, symbols)

    # reset substitution args
    substitution_args_context['arg'] = {}


def open_output(output_filename):
    if output_filename is None:
        return sys.stdout
    else:
        dir_name = os.path.dirname(output_filename)
        if dir_name and not os.path.isdir(dir_name): os.makedirs(dir_name)
        return open(output_filename, 'w')


def main():
    opts, input_file = process_cli_args(sys.argv[1:])
    try:
        doc = parse(None, input_file)
        process_doc(doc, filename=input_file, **vars(opts))

    except xml.parsers.expat.ExpatError as e:
        print("XML parsing error: ", str(e), file=sys.stderr)
        print("Check that:", file=sys.stderr)
        print(" - Your XML is well-formed", file=sys.stderr)
        print(" - You have the xacro xmlns declaration:",
              "xmlns:xacro=\"http://www.ros.org/wiki/xacro\"", file=sys.stderr)
        sys.exit(2) # indicate failure, but don't print stack trace on XML errors

    except Exception as e:
        print(file=sys.stderr) # add empty separator line before error
        if opts.debug:
            raise # create stack trace
        else:
            print('{name}: {msg}'.format(name=type(e).__name__, msg=str(e)),
                  file=sys.stderr)
            sys.exit(2) # indicate failure, but don't print stack trace on XML errors

    out = open_output(opts.output)

    if opts.just_deps:
        out.write(" ".join(all_includes))
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
    if opts.output: out.close()
