#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
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
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
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


import os.path, sys, os, getopt
import subprocess
from xml.dom.minidom import parse, parseString
import xml.dom
import re

TAG_PREFIX = ''  # 'xacro:'

class XacroException(Exception): pass

def isnumber(x):
    return hasattr(x, '__int__')

import roslib; roslib.load_manifest('xacro')
# Suppresses the output from roslaunch by killing the logger
import roslib.scriptutil
def return_none(a=0,b=0): return None
roslib.scriptutil.configure_logging = return_none
import roslaunch
def eval_extension(str):
    return roslaunch.core.resolve_args(str)

class Table:
    def __init__(self, parent = None):
        self.parent = parent
        self.table = {}

    def __getitem__(self, key):
        if key in self.table:
            return self.table[key]
        elif self.parent:
            return self.parent[key]
        else:
            raise KeyError(key)

    def __setitem__(self, key, value):
        self.table[key] = value

    def __contains__(self, key):
        return \
            key in self.table or \
            (self.parent and key in self.parent)


class QuickLexer(object):
    def __init__(self, **res):
        self.str = ""
        self.top = None
        self.res = []
        for k,v in res.items():
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
    if child: return child
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

def child_elements(elt):
    c = elt.firstChild
    while c:
        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
            yield c
        c = c.nextSibling

all_includes = []
## @throws XacroException if a parsing error occurs with an included document
def process_includes(doc, base_dir):
    previous = doc.documentElement
    elt = next_element(previous)
    while elt:
        if elt.tagName == 'include':
            filename = eval_text(elt.getAttribute('filename'), {})
            if not os.path.isabs(filename):
                filename = os.path.join(base_dir, filename)
            f = None
            try:
                try:
                    f = open(filename)
                except IOError, e:
                    print elt
                    raise XacroException("included file \"%s\" could not be opened: %s" % (filename, str(e)))
                try:
                    global all_includes
                    all_includes.append(filename)
                    included = parse(f)
                except Exception, e:
                    raise XacroException("included file [%s] generated an error during XML parsing: %s"%(filename, str(e)))
            finally:
                if f: f.close()

            # Replaces the include tag with the elements of the included file
            for c in child_elements(included.documentElement):
                elt.parentNode.insertBefore(c.cloneNode(1), elt)
            elt.parentNode.removeChild(elt)
            elt = None
        else:
            previous = elt

        elt = next_element(previous)

# Returns a dictionary: { macro_name => macro_xml_block }
def grab_macros(doc):
    macros = {}

    previous = doc.documentElement
    elt = next_element(previous)
    while elt:
        if elt.tagName == TAG_PREFIX + 'macro':
            name = elt.getAttribute('name')

            macros[name] = elt

            elt.parentNode.removeChild(elt)
            elt = None
        else:
            previous = elt

        elt = next_element(previous)
    return macros

# Returns a Table of the properties
def grab_properties(doc):
    table = Table()

    previous = doc.documentElement
    elt = next_element(previous)
    while elt:
        if elt.tagName == TAG_PREFIX + 'property':
            name = elt.getAttribute('name')
            value = None

            if elt.hasAttribute('value'):
                value = elt.getAttribute('value')
            else:
                name = '*' + name
                value = elt

            table[name] = value

            elt.parentNode.removeChild(elt)
            elt = None
        else:
            previous = elt

        elt = next_element(previous)
    return table

def eat_ignore(lex):
    while lex.peek() and lex.peek()[0] == lex.IGNORE:
        lex.next()

def eval_lit(lex, symbols):
    eat_ignore(lex)
    if lex.peek()[0] == lex.NUMBER:
        return float(lex.next()[1])
    if lex.peek()[0] == lex.SYMBOL:
        try:
            value = symbols[lex.next()[1]]
        except KeyError, ex:
            sys.stderr.write("Could not find symbol: %s\n" % str(ex))
            return 0
        if not (isnumber(value) or isinstance(value,(str,unicode))):
            print [value], isinstance(value, str), type(value)
            raise "WTF2"
        try:
            return float(value)
        except:
            return value
    raise "Bad literal"

def eval_factor(lex, symbols):
    eat_ignore(lex)

    neg = 1;
    if lex.peek()[1] == '-':
        lex.next()
        neg = -1

    if lex.peek()[0] in [lex.NUMBER, lex.SYMBOL]:
        return neg * eval_lit(lex, symbols)
    if lex.peek()[0] == lex.LPAREN:
        lex.next()
        eat_ignore(lex)
        result = eval_expr(lex, symbols)
        eat_ignore(lex)
        if lex.next()[0] != lex.RPAREN:
            raise "Unmatched left paren"
        eat_ignore(lex)
        return neg * result

    raise "Misplaced operator"

def eval_term(lex, symbols):
    eat_ignore(lex)

    result = 0
    if lex.peek()[0] in [lex.NUMBER, lex.SYMBOL, lex.LPAREN] \
            or lex.peek()[1] == '-':
        result = eval_factor(lex, symbols)

    eat_ignore(lex)
    while lex.peek() and lex.peek()[1] in ['*', '/']:
        op = lex.next()[1]
        n = eval_factor(lex, symbols)

        if op == '*':
            result = float(result) * float(n)
        elif op == '/':
            result = float(result) / float(n)
        else:
            raise "WTF"
        eat_ignore(lex)
    return result

def eval_expr(lex, symbols):
    eat_ignore(lex)

    op = None
    if lex.peek()[0] == lex.OP:
        op = lex.next()[1]
        if not op in ['+', '-']:
            raise "Invalid op"

    result = eval_term(lex, symbols)
    if op == '-':
        result = -float(result)

    eat_ignore(lex)
    while lex.peek() and lex.peek()[1] in ['+', '-']:
        op = lex.next()[1]
        n = eval_term(lex, symbols)

        if op == '+':
            result = float(result) + float(n)
        if op == '-':
            result = float(result) - float(n)
        eat_ignore(lex)
    return result


def eval_text(text, symbols):
    def handle_expr(s):
        lex = QuickLexer(IGNORE = r"\s+",
                         NUMBER = r"(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?",
                         SYMBOL = r"[a-zA-Z_]\w*",
                         OP = r"[\+\-\*/^]",
                         LPAREN = r"\(",
                         RPAREN = r"\)")
        lex.lex(s)
        return eval_expr(lex, symbols)
    def handle_extension(s):
        return eval_extension("$(%s)" % s)

    results = []
    lex = QuickLexer(EXPR = r"\$\{[^\}]*\}",
                     EXTENSION = r"\$\([^\)]*\)",
                     TEXT = r"[^\$]+")
    lex.lex(text)
    while lex.peek():
        if lex.peek()[0] == lex.EXPR:
            results.append(handle_expr(lex.next()[1][2:-1]))
        elif lex.peek()[0] == lex.EXTENSION:
            results.append(handle_extension(lex.next()[1][2:-1]))
        elif lex.peek()[0] == lex.TEXT:
            results.append(lex.next()[1])
    return ''.join(map(str, results))

# Expands macros, replaces properties, and evaluates expressions
def eval_all(root, macros, symbols):
    previous = root
    node = next_node(previous)
    while node:
        if node.nodeType == xml.dom.Node.ELEMENT_NODE:
            if node.tagName in macros:
                body = macros[node.tagName].cloneNode(deep = True)
                params = body.getAttribute('params').split()

                # Expands the macro
                scoped = Table(symbols)
                for name,value in node.attributes.items():
                    if not name in params:
                        raise "Invalid parameter \"%s\" while expanding macro \"%s\"" % \
                            (str(name), str(node.tagName))
                    params.remove(name)
                    scoped[name] = eval_text(value, symbols)
                if len(params) == 1 and params[0][0] == '*':
                    block = node.cloneNode(deep = True)
                    eval_all(block, macros, symbols)
                    scoped[params[0]] = block
                    params = []
                if params:
                    raise "Some parameters were not set for macro %s" % \
                        str(node.tagName)
                eval_all(body, macros, scoped)

                # Replaces the macro node with the expansion
                for e in list(child_elements(body)):  # Ew
                    node.parentNode.insertBefore(e, node)
                node.parentNode.removeChild(node)

                node = None
            elif node.tagName == TAG_PREFIX + 'insert_block':
                name = node.getAttribute('name')
                block = symbols['*' + name]

                for e in list(child_elements(block)):
                    node.parentNode.insertBefore(e, node)
                node.parentNode.removeChild(node)

                node = None
            else:
                # Evals the attributes
                for at in node.attributes.items():
                    result = eval_text(at[1], symbols)
                    node.setAttribute(at[0], result)
                previous = node
        elif node.nodeType == xml.dom.Node.TEXT_NODE:
            node.data = eval_text(node.data, symbols)
            previous = node
        else:
            previous = node

        node = next_node(previous)
    return macros

# Expands everything except includes
def eval_self_contained(doc):
    macros = grab_macros(doc)
    symbols = grab_properties(doc)
    eval_all(doc.documentElement, macros, symbols)

def print_usage(exit_code = 0):
    print "Usage: %s [-o <output>] <input>" % 'xacro.py'
    print "       %s --deps" % 'xacro.py'
    sys.exit(exit_code)

if __name__ == '__main__':

    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], "ho:", ['deps'])
    except getopt.GetoptError, err:
        print str(err)
        print_usage(2)

    just_deps = False

    output = sys.stdout
    for o, a in opts:
        if o == '-h':
            print_usage(0)
        elif o == '-o':
            output = open(a, 'w')
        elif o == '--deps':
            just_deps = True

    if len(args) < 1:
        print "No input given"
        print_usage(2)

    f = open(args[0])
    doc = parse(f)
    f.close()


    process_includes(doc, os.path.dirname(sys.argv[1]))
    if just_deps:
        for inc in all_includes:
            print inc
    else:
        eval_self_contained(doc)
        doc.writexml(output)
        print
