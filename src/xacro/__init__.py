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

import getopt
import glob
import os
import re
import string
import sys
import xml
import ast

from xml.dom.minidom import parse

from roslaunch import substitution_args
from rosgraph.names import load_mappings

try:
    _basestr = basestring
except NameError:
    _basestr = str

# Dictionary of subtitution args
substitution_args_context = {}


class XacroException(Exception):
    pass


def eval_extension(str):
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

    def _resolve_(self,key):
        # lazy evaluation
        if key in self.unevaluated:
            if key in self.recursive:
                raise XacroException("recursive variable definition: %s" %
                                     " -> ".join(self.recursive + [key]))
            self.recursive.append(key)
            self.table[key] = eval_text(self.table[key], self)
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
        # TODO: does this still work in python3 ?
        if isinstance(value, basestring):
            try:
                # try to evaluate as literal, e.g. number, boolean, etc.
                value = ast.literal_eval(value)
            except:
                # otherwise simply store as original string for later (re)evaluation
                pass

        self.table[key] = value
        if isinstance(value, basestring):
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
        if isinstance (self.parent, Table):
            s += "\n  parent: "
            s += str(self.parent)
        return s


class QuickLexer(object):
    def __init__(self, **res):
        self.str = ""
        self.top = None
        self.res = []
        for k, v in res.items():
            self.__setattr__(k, len(self.res))
            self.res.append(v)

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
            m = re.match(self.res[i], self.str)
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
basedir="."

# Deprecated message for <include> tags that don't have <xacro:include> prepended:
deprecated_include_msg = """DEPRECATED IN HYDRO:
  The <include> tag should be prepended with 'xacro' if that is the intended use
  of it, such as <xacro:include ...>. Use the following script to fix incorrect
  xacro includes:
     sed -i 's/<include/<xacro:include/g' `find . -iname *.xacro`"""

include_no_matches_msg = """Include tag filename spec \"{}\" matched no files."""


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

def process_include(elt, symbols):
    namespaces = {}
    filename_spec = eval_text(elt.getAttribute('filename'), symbols)
    if not os.path.isabs(filename_spec):
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
            with open(filename) as f:
                try:
                    included = parse(f)
                except Exception as e:
                    raise XacroException(
                        "included file \"%s\" generated an error during XML parsing: %s"
                        % (filename, str(e)))
        except IOError as e:
            raise XacroException("included file \"%s\" could not be opened: %s" % (filename, str(e)))

        # Replaces the include tag with the nodes of the included file
        for c in child_nodes(included.documentElement):
            elt.parentNode.insertBefore(c.cloneNode(deep=True), elt)

        # Grabs all the declared namespaces of the included document
        for name, value in included.documentElement.attributes.items():
            if name.startswith('xmlns:'):
                namespaces[name] = value

    # Makes sure the final document declares all the namespaces of the included documents.
    for k, v in namespaces.items():
        elt.parentNode.setAttribute(k, v)

    elt.parentNode.removeChild(elt)

## @throws XacroException if a parsing error occurs with an included document
def process_includes(doc, dir=None):
    global basedir
    if dir: basedir = dir

    previous = doc.documentElement
    elt = next_element(previous)
    while elt:
        if is_include(elt):
            process_include(elt, {})
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

# evaluate text and return typed value
def eval_text(text, symbols):
    def handle_expr(s):
        try:
            # taking simple security measures to forbid access to __builtins__
            # for discussion, see: http://nedbatchelder.com/blog/201206/eval_really_is_dangerous.html
            return eval(s, {'__builtins__':{}}, symbols)
        except NameError as e:
            raise XacroException("%s evaluating expression '%s'" % (str(e), s))
        except Exception as e:
            raise

    def handle_extension(s):
        return eval_extension("$(%s)" % s)

    results = []
    lex = QuickLexer(DOLLAR_DOLLAR_BRACE=r"\$\$+\{",
                     EXPR=r"\$\{[^\}]*\}",
                     EXTENSION=r"\$\([^\)]*\)",
                     TEXT=r"([^\$]|\$[^{(]|\$$)+")
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
    if (len(results) == 1): return results[0]
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

# Expands macros, replaces properties, and evaluates expressions
def eval_all(root, macros={}, symbols=Table()):
    # Evaluates the attributes for the root node
    for at in root.attributes.items():
        result = str(eval_text(at[1], symbols))
        root.setAttribute(at[0], result)

    previous = root
    node = next_node(previous)
    while node:
        if node.nodeType == xml.dom.Node.ELEMENT_NODE:
            if is_include(node):
                process_include(node, symbols)
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
                eval_all(cloned, macros, symbols)
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
                eval_all(body, macros, scoped)

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
                value = eval_text(node.getAttribute('value'), symbols)
                try:
                    # try to interpret value as boolean
                    if isinstance(value, basestring):
                        if   value == "true": keep = True
                        elif value == "false": keep = False
                        else: keep = ast.literal_eval(value)
                    else: keep = bool(value)
                except:
                    raise XacroException("Xacro conditional \"%s\" evaluated to \"%s\", which is not a boolean expression." % (node.getAttribute('value'), value))
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


def eval_self_contained(doc, in_order=False):
    import math;
    symbols = {}
    symbols.update(math.__dict__)

    if not in_order:
        # process includes, macros, and properties before evaluating stuff
        process_includes(doc)
        macros = grab_macros(doc)
        symbols = grab_properties(doc, Table(symbols))
    else:
        macros  = {}
        symbols = Table(symbols)

    eval_all(doc.documentElement, macros, symbols)

def print_usage(exit_code=0):
    print("Usage: %s [-o <output>] <input>" % 'xacro.py')
    print("       %s --deps       Prints dependencies" % 'xacro.py')
    print("       %s --includes   Only evalutes includes" % 'xacro.py')
    sys.exit(exit_code)


def set_substitution_args_context(context={}):
    substitution_args_context['arg'] = context

def open_output(output_filename):
    if output_filename is None:
        return sys.stdout
    else:
        return open(output_filename, 'w')

def main():
    global basedir

    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], "ho:",
                                       ['deps', 'includes', 'inorder'])
    except getopt.GetoptError as err:
        print(str(err))
        print_usage(2)

    just_deps = False
    just_includes = False
    in_order = False

    output_filename = None
    for o, a in opts:
        if o == '-h':
            print_usage(0)
        elif o == '-o':
            output_filename = a
        elif o == '--deps':
            just_deps = True
        elif o == '--includes':
            just_includes = True
        elif o == '--inorder':
            in_order = True

    if len(args) < 1:
        print("No input given")
        print_usage(2)

    # Process substitution args
    set_substitution_args_context(load_mappings(sys.argv))

    f = open(args[0])
    doc = None
    try:
        doc = parse(f)
    except xml.parsers.expat.ExpatError:
        sys.stderr.write("Expat parsing error.  Check that:\n")
        sys.stderr.write(" - Your XML is correctly formed\n")
        sys.stderr.write(" - You have the xacro xmlns declaration: " +
                         "xmlns:xacro=\"http://www.ros.org/wiki/xacro\"\n")
        sys.stderr.write("\n")
        raise
    finally:
        f.close()

    basedir = os.path.dirname(args[0])
    if just_deps or just_includes:
        process_includes(doc)
        if just_deps:
            sys.stdout.write(" ".join(all_includes))
            sys.stdout.write("\n")
        if just_includes:
            open_output(output_filename).write(doc.toprettyxml(indent='  '))
            print()
        return

    eval_self_contained(doc, in_order)

    banner = [xml.dom.minidom.Comment(c) for c in
              [" %s " % ('=' * 83),
               " |    This document was autogenerated by xacro from %-30s | " % args[0],
               " |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED  %-30s | " % "",
               " %s " % ('=' * 83)]]
    first = doc.firstChild
    for comment in banner:
        doc.insertBefore(comment, first)

    open_output(output_filename).write(doc.toprettyxml(indent='  '))
    print()
