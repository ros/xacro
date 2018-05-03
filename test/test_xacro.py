#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import unittest
import xacro
from xml.dom.minidom import parseString
import xml.dom
import os.path
import tempfile
import shutil
import subprocess
import re
import ast
try:
    from cStringIO import StringIO # Python 2.x
except ImportError:
    from io import StringIO # Python 3.x
from contextlib import contextmanager


# regex to match whitespace
whitespace = re.compile(r'\s+')

def text_values_match(a, b):
    # generic comparison
    if whitespace.sub(' ', a).strip() == whitespace.sub(' ', b).strip():
        return True

    try: # special handling of dicts: ignore order
        a_dict = ast.literal_eval(a)
        b_dict = ast.literal_eval(b)
        if (isinstance(a_dict, dict) and isinstance(b_dict, dict) and a_dict == b_dict):
            return True
    except:  # Attribute values aren't dicts
        pass

    # on failure, try to split a and b at whitespace and compare snippets
    def match_splits(a_, b_):
        if len(a_) != len(b_): return False
        for a, b in zip(a_, b_):
            if a == b: continue
            try:  # compare numeric values only up to some accuracy
                if abs(float(a) - float(b)) > 1.0e-9:
                    return False
            except ValueError:  # values aren't numeric and not identical
                return False
        return True

    return match_splits(a.split(), b.split())


def all_attributes_match(a, b):
    if len(a.attributes) != len(b.attributes):
        print("Different number of attributes")
        return False
    a_atts = a.attributes.items()
    b_atts = b.attributes.items()
    a_atts.sort()
    b_atts.sort()

    for a, b in zip(a_atts, b_atts):
        if a[0] != b[0]:
            print("Different attribute names: %s and %s" % (a[0], b[0]))
            return False
        if not text_values_match(a[1], b[1]):
            print("Different attribute values: %s and %s" % (a[1], b[1]))
            return False
    return True


def text_matches(a, b):
    if text_values_match(a, b): return True
    print("Different text values: '%s' and '%s'" % (a, b))
    return False


def nodes_match(a, b, ignore_nodes):
    if not a and not b:
        return True
    if not a or not b:
        return False

    if a.nodeType != b.nodeType:
        print("Different node types: %s and %s" % (a, b))
        return False

    # compare text-valued nodes
    if a.nodeType in [xml.dom.Node.TEXT_NODE,
                      xml.dom.Node.CDATA_SECTION_NODE,
                      xml.dom.Node.COMMENT_NODE]:
        return text_matches(a.data, b.data)

    # ignore all other nodes except ELEMENTs
    if a.nodeType != xml.dom.Node.ELEMENT_NODE:
        return True

    # compare ELEMENT nodes
    if a.nodeName != b.nodeName:
        print("Different element names: %s and %s" % (a.nodeName, b.nodeName))
        return False

    if not all_attributes_match(a, b):
        return False

    a = a.firstChild
    b = b.firstChild
    while a or b:
        # ignore whitespace-only text nodes
        # we could have several text nodes in a row, due to replacements
        while (a and 
               ((a.nodeType in ignore_nodes) or
                (a.nodeType == xml.dom.Node.TEXT_NODE and whitespace.sub('', a.data) == ""))):
            a = a.nextSibling
        while (b and 
               ((b.nodeType in ignore_nodes) or
                (b.nodeType == xml.dom.Node.TEXT_NODE and whitespace.sub('', b.data) == ""))):
            b = b.nextSibling

        if not nodes_match(a, b, ignore_nodes):
            return False

        if a: a = a.nextSibling
        if b: b = b.nextSibling

    return True


def xml_matches(a, b, ignore_nodes=[]):
    if isinstance(a, str):
        return xml_matches(parseString(a).documentElement, b, ignore_nodes)
    if isinstance(b, str):
        return xml_matches(a, parseString(b).documentElement, ignore_nodes)
    if a.nodeType == xml.dom.Node.DOCUMENT_NODE:
        return xml_matches(a.documentElement, b, ignore_nodes)
    if b.nodeType == xml.dom.Node.DOCUMENT_NODE:
        return xml_matches(a, b.documentElement, ignore_nodes)

    if not nodes_match(a, b, ignore_nodes):
        print("Match failed:")
        a.writexml(sys.stdout)
        print()
        print('=' * 78)
        b.writexml(sys.stdout)
        print()
        return False
    return True


# capture output going to file=sys.stdout | sys.stderr
@contextmanager
def capture_stderr(function, *args, **kwargs):
  old, sys.stderr = sys.stderr, StringIO()  # temporarily replace sys.stderr with StringIO()
  result = function(*args, **kwargs)
  sys.stderr.seek(0)
  yield (result, sys.stderr.read())
  sys.stderr = old  # restore sys.stderr


class TestMatchXML(unittest.TestCase):
    def test_normalize_whitespace_text(self):
        self.assertTrue(text_matches("", " \t\n\r"))
    def test_normalize_whitespace_trim(self):
        self.assertTrue(text_matches(" foo bar ", "foo \t\n\r bar"))

    def test_match_similar_numbers(self):
        self.assertTrue(text_matches("0.123456789", "0.123456788"))
    def test_mismatch_different_numbers(self):
        self.assertFalse(text_matches("0.123456789", "0.1234567879"))

    def test_match_unordered_dicts(self):
        self.assertTrue(text_matches("{'a': 1, 'b': 2, 'c': 3}", "{'c': 3, 'b': 2, 'a': 1}"))
    def test_mismatch_different_dicts(self):
        self.assertFalse(text_matches("{'a': 1, 'b': 2, 'c': 3}", "{'c': 3, 'b': 2, 'a': 0}"))

    def test_empty_node_vs_whitespace(self):
        self.assertTrue(xml_matches('''<foo/>''', '''<foo> \t\n\r </foo>'''))
    def test_whitespace_vs_empty_node(self):
        self.assertTrue(xml_matches('''<foo> \t\n\r </foo>''', '''<foo/>'''))
    def test_normalize_whitespace_nested(self):
        self.assertTrue(xml_matches('''<a><b/></a>''', '''<a>\n<b> </b> </a>'''))

    def test_ignore_comments(self):
        self.assertTrue(xml_matches('''<a><b/><!-- foo --> <!-- bar --></a>''',
                                    '''<a><b/></a>''', [xml.dom.Node.COMMENT_NODE]))


class TestXacroFunctions(unittest.TestCase):
    def test_is_valid_name(self):
        self.assertTrue(xacro.is_valid_name("_valid_name_123"))
        self.assertFalse(xacro.is_valid_name('pass'))     # syntactically correct keyword
        self.assertFalse(xacro.is_valid_name('foo '))     # trailing whitespace
        self.assertFalse(xacro.is_valid_name(' foo'))     # leading whitespace
        self.assertFalse(xacro.is_valid_name('1234'))     # number
        self.assertFalse(xacro.is_valid_name('1234abc'))  # number and letters
        self.assertFalse(xacro.is_valid_name(''))         # empty string
        self.assertFalse(xacro.is_valid_name('   '))      # whitespace only
        self.assertFalse(xacro.is_valid_name('foo bar'))  # several tokens
        self.assertFalse(xacro.is_valid_name('no-dashed-names-for-you'))
        self.assertFalse(xacro.is_valid_name('invalid.too'))  # dot separates fields

    def test_resolve_macro(self):
        # define three nested macro dicts with the same macro names (keys)
        content = {'xacro:simple': 'simple'}
        ns2 = dict({k: v+'2' for k,v in content.items()})
        ns1 = dict({k: v+'1' for k,v in content.items()})
        ns1.update(ns2=ns2)
        macros = dict(content)
        macros.update(ns1=ns1)

        self.assertEqual(xacro.resolve_macro('simple', macros), 'simple')
        self.assertEqual(xacro.resolve_macro('ns1.simple', macros), 'simple1')
        self.assertEqual(xacro.resolve_macro('ns1.ns2.simple', macros), 'simple2')

        self.assertEqual(xacro.resolve_macro('xacro:simple', macros), 'simple')
        self.assertEqual(xacro.resolve_macro('xacro:ns1.simple', macros), 'simple1')
        self.assertEqual(xacro.resolve_macro('xacro:ns1.ns2.simple', macros), 'simple2')

    def check_macro_arg(self, s, param, forward, default, rest):
        p, v, r = xacro.parse_macro_arg(s)
        self.assertEqual(p, param, msg="'{0}' != '{1}' parsing {2}".format(p, param, s))
        if forward or default:
            self.assertTrue(v is not None)
            self.assertEqual(v[0], forward, msg="'{0}' != '{1}' parsing {2}".format(v[0], forward, s))
            self.assertEqual(v[1], default, msg="'{0}' != '{1}' parsing {2}".format(v[1], default, s))
        else:
            self.assertTrue(v is None)
        self.assertEqual(r, rest, msg="'{0}' != '{1}' parsing {2}".format(r, rest, s))

    def test_parse_macro_arg(self):
        for forward in ['', '^', '^|']:
            defaults = ['', "f('some string','some other')", "f('a b')"]
            if forward == '^': defaults = ['']
            for default in defaults:
                seps = ['=', ':='] if forward or default else ['']
                for sep in seps:
                    for rest in ['', ' ', ' bar', ' bar=42']:
                        s = 'foo{0}{1}{2}{3}'.format(sep, forward, default, rest)
                        self.check_macro_arg(s, 'foo', 'foo' if forward else None,
                                             default if default else None,
                                             rest.lstrip())
    def test_parse_macro_whitespace(self):
        for ws in ['  ', ' \t ', ' \n ']:
            self.check_macro_arg(ws + 'foo' + ws + 'bar=42' + ws, 'foo', None, None, 'bar=42' + ws)

# base class providing some convenience functions
class TestXacroBase(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestXacroBase, self).__init__(*args, **kwargs)
        self.in_order = False
        self.ignore_nodes = []

    def assert_matches(self, a, b):
        self.assertTrue(xml_matches(a, b, self.ignore_nodes))

    def quick_xacro(self, xml, cli=None, **kwargs):
        args = {}
        if cli:
            opts, _ = xacro.cli.process_args(cli, require_input=False)
            args.update(vars(opts))  # initialize with cli args
        args.update(dict(in_order = self.in_order))  # set in_order option from test class
        args.update(kwargs)  # explicit function args have highest priority

        doc = xacro.parse(xml)
        xacro.process_doc(doc, **args)
        return doc

    def run_xacro(self, input_path, *args):
        args = list(args)
        if not self.in_order:
            args.append('--legacy')
        test_dir = os.path.abspath(os.path.dirname(__file__))
        xacro_path = os.path.join(test_dir, '..', 'scripts', 'xacro')
        subprocess.call([xacro_path, input_path] + args)


# class to match XML docs while ignoring any comments
class TestXacroCommentsIgnored(TestXacroBase):
    def __init__(self, *args, **kwargs):
        super(TestXacroCommentsIgnored, self).__init__(*args, **kwargs)
        self.ignore_nodes = [xml.dom.Node.COMMENT_NODE]

    def test_pr2(self):
        # run xacro on the pr2 tree snapshot
        test_dir= os.path.abspath(os.path.dirname(__file__))
        pr2_xacro_path = os.path.join(test_dir, 'robots', 'pr2', 'pr2.urdf.xacro')
        pr2_golden_parse_path = os.path.join(test_dir, 'robots', 'pr2', 'pr2_1.11.4.xml')
        self.assert_matches(
                xml.dom.minidom.parse(pr2_golden_parse_path),
                self.quick_xacro(open(pr2_xacro_path)))


# standard test class (including the test from TestXacroCommentsIgnored)
class TestXacro(TestXacroCommentsIgnored):
    def __init__(self, *args, **kwargs):
        super(TestXacroCommentsIgnored, self).__init__(*args, **kwargs)
        self.ignore_nodes = []

    def test_invalid_property_name(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:property name="invalid.name"/></a>'''
        self.assertRaises(xacro.XacroException, self.quick_xacro, src)

    def test_dynamic_macro_names(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo"><a>foo</a></xacro:macro>
  <xacro:macro name="bar"><b>bar</b></xacro:macro>
  <xacro:property name="var" value="%s"/>
  <xacro:call macro="${var}"/></a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">%s</a>'''
        self.assert_matches(self.quick_xacro(src % "foo"), res % "<a>foo</a>")
        self.assert_matches(self.quick_xacro(src % "bar"), res % "<b>bar</b>")

    def test_dynamic_macro_name_clash(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo"><a name="foo"/></xacro:macro>
  <xacro:macro name="call"><a name="bar"/></xacro:macro>
  <xacro:call/></a>'''
        # for now we only issue a deprecated warning and expect the old behaviour
        # resolving macro "call"
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><a name="bar"/></a>'''
        # new behaviour would be to resolve to foo of course
        # res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><a name="foo"/></a>'''
        with capture_stderr(self.quick_xacro, src) as (result, output):
            self.assert_matches(result, res)
            self.assertTrue("deprecated use of macro name 'call'" in output)

    def test_dynamic_macro_undefined(self):
        self.assertRaises(xacro.XacroException,
                          self.quick_xacro,
                          '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                          <xacro:call macro="foo"/></a>''')

    def test_macro_undefined(self):
        self.assertRaises(xacro.XacroException,
                          self.quick_xacro,
                          '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                          <xacro:undefined><foo/><bar/></xacro:undefined></a>''')

    def test_xacro_element(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="name"><xacro:element xacro:name="${name}"/></xacro:macro>
  <xacro:foo name="A"/>
  <xacro:foo name="B"/>
</a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><A/><B/></a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_xacro_attribute(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="name value">
  <tag><xacro:attribute name="${name}" value="${value}"/></tag>
  </xacro:macro>
  <xacro:foo name="A" value="foo"/>
  <xacro:foo name="B" value="bar"/>
</a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><tag A="foo"/><tag B="bar"/></a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_inorder_processing(self):
        src = '''<xml xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="1.0"/>
  <xacro:macro name="m" params="foo"><a foo="${foo}"/></xacro:macro>
  <xacro:m foo="1 ${foo}"/>
  <!-- now redefining the property and macro -->
  <xacro:property name="foo" value="2.0"/>
  <xacro:macro name="m" params="foo"><b bar="${foo}"/></xacro:macro>
  <xacro:m foo="2 ${foo}"/>
</xml>'''
        oldOrder = '''
<xml xmlns:xacro="http://www.ros.org/wiki/xacro">
  <b bar="1 2.0"/>
  <b bar="2 2.0"/>
</xml>
'''
        inOrder = '''
<xml xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a foo="1 1.0"/>
  <b bar="2 2.0"/>
</xml>
'''
        self.assert_matches(self.quick_xacro(src), inOrder if self.in_order else oldOrder)

    def test_should_replace_before_macroexpand(self):
        self.assert_matches(
                self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="inner" params="*the_block">
  <in_the_inner><xacro:insert_block name="the_block" /></in_the_inner>
</xacro:macro>
<xacro:macro name="outer" params="*the_block">
  <in_the_outer><xacro:inner><xacro:insert_block name="the_block" /></xacro:inner></in_the_outer>
</xacro:macro>
<xacro:outer><woot /></xacro:outer></a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<in_the_outer><in_the_inner><woot /></in_the_inner></in_the_outer></a>''')

    def test_evaluate_macro_params_before_body(self):
        self.assert_matches(self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="lst">${lst[-1]}</xacro:macro>
  <foo lst="${[1,2,3]}"/></a>'''),
        '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">3</a>''')

    def test_macro_params_escaped_string(self):
        self.assert_matches(self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="foo" params="a='1 -2' c=3"><bar a="${a}" c="${c}"/></xacro:macro>
    <xacro:foo/></a>'''),
                            '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><bar a="1 -2" c="3"/></a>''')

    def test_property_replacement(self):
        self.assert_matches(
                self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="42" />
  <the_foo result="${foo}" />
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <the_foo result="42" />
</a>''')

    def test_property_scope_parent(self):
        self.assert_matches(self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="factor">
  <xacro:property name="foo" value="${21*factor}" scope="parent"/>
  </xacro:macro>
  <xacro:foo factor="2"/><a foo="${foo}"/></a>'''),
        '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><a foo="42"/></a>''')

    def test_property_scope_global(self):
        self.assert_matches(self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="factor">
    <xacro:macro name="bar">
      <xacro:property name="foo" value="${21*factor}" scope="global"/>
    </xacro:macro>
    <xacro:bar/>
  </xacro:macro>
  <xacro:foo factor="2"/><a foo="${foo}"/></a>'''),
        '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><a foo="42"/></a>''')

    def test_math_ignores_spaces(self):
        self.assert_matches(
                self.quick_xacro('''<a><f v="${0.9 / 2 - 0.2}" /></a>'''),
                '''<a><f v="0.25" /></a>''')

    def test_substitution_args_find(self):
        self.assert_matches(
                self.quick_xacro('''<a><f v="$(find xacro)/test/test_xacro.py" /></a>'''),
                '''<a><f v="''' + os.path.abspath((__file__).replace(".pyc",".py") + '''" /></a>'''))

    def test_substitution_args_arg(self):
        self.assert_matches(
                self.quick_xacro('''<a><f v="$(arg sub_arg)" /></a>''', cli=['sub_arg:=my_arg']),
                '''<a><f v="my_arg" /></a>''')

    def test_escaping_dollar_braces(self):
        self.assert_matches(
                self.quick_xacro('''<a b="$${foo}" c="$$${foo}" d="text $${foo}" e="text $$${foo}" f="$$(pwd)" />'''),
                '''<a b="${foo}" c="$${foo}" d="text ${foo}" e="text $${foo}" f="$(pwd)" />''')

    def test_just_a_dollar_sign(self):
        self.assert_matches(
                self.quick_xacro('''<a b="$" c="text $" d="text $ text"/>'''),
                '''<a b="$" c="text $" d="text $ text"/>''')

    def test_multiple_insert_blocks(self):
        self.assert_matches(
                self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="foo" params="*block">
  <xacro:insert_block name="block" />
  <xacro:insert_block name="block" />
</xacro:macro>
<xacro:foo>
  <a_block />
</xacro:foo>
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a_block />
  <a_block />
</a>''')

    def test_multiple_blocks(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="foo" params="*block{A} *block{B}">
  <xacro:insert_block name="block1" />
  <xacro:insert_block name="block2" />
</xacro:macro>
<xacro:foo>
  <block1/>
  <block2/>
</xacro:foo>
</a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<block{A}/>
<block{B}/>
</a>'''
        # test both, reversal and non-reversal of block order
        for d in [dict(A='1', B='2'), dict(A='2', B='1')]:
            self.assert_matches(self.quick_xacro(src.format(**d)), res.format(**d))

    def test_integer_stays_integer(self):
        self.assert_matches(
                self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="m" params="num">
  <test number="${num}" />
</xacro:macro>
<xacro:m num="100" />
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <test number="100" />
</a>''')

    def test_insert_block_property(self):
        self.assert_matches(
                self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="bar">bar</xacro:macro>
<xacro:property name="val" value="2" />
<xacro:property name="some_block">
  <some_block attr="${val}"><xacro:bar/></some_block>
</xacro:property>
<foo>
  <xacro:insert_block name="some_block" />
</foo>
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<foo><some_block attr="2">bar</some_block></foo>
</a>''')

    def test_include(self):
        self.assert_matches(self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/xacro">
  <xacro:include filename="include1.xml" /></a>'''),
                        '''<a xmlns:xacro="http://www.ros.org/xacro"><inc1/></a>''')

    def test_include_glob(self):
        input  = '''<a xmlns:xacro="http://www.ros.org/xacro">
                    <xacro:include filename="include{glob}.xml"/></a>'''
        result = '<a xmlns:xacro="http://www.ros.org/xacro"><inc1/><inc2/></a>'
        for pattern in ['*', '?', '[1-2]']:
            self.assert_matches(self.quick_xacro(input.format(glob=pattern)), result)

    def test_include_nonexistent(self):
        self.assertRaises(xacro.XacroException,
                          self.quick_xacro, '''<a xmlns:xacro="http://www.ros.org/xacro">
                             <xacro:include filename="include-nada.xml" /></a>''')

    def test_include_deprecated(self):
        # <include> tags with some non-trivial content should not issue the deprecation warning
        src = '''<a><include filename="nada"><tag/></include></a>'''
        with capture_stderr(self.quick_xacro, src) as (result, output):
            self.assert_matches(result, src)
            self.assertEqual(output, '')

    def test_include_from_variable(self):
        doc = '''<a xmlns:xacro="http://www.ros.org/xacro">
        <xacro:property name="file" value="include1.xml"/>
        <xacro:include filename="${file}" /></a>'''
        if self.in_order:
            self.assert_matches(self.quick_xacro(doc),
                '''<a xmlns:xacro="http://www.ros.org/xacro"><inc1/></a>''')
        else:
            self.assertRaises(xacro.XacroException, self.quick_xacro, doc)

    def test_include_recursive(self):
        self.assert_matches(self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/xacro">
    <xacro:include filename="include1.xml"/>
    <xacro:include filename="./include1.xml"/>
    <xacro:include filename="subdir/include-recursive.xacro"/>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/xacro">
<inc1/><inc1/>
<subdir_inc1/><subdir_inc1/><inc1/></a>''')

    def test_include_with_namespace(self):
        doc = '''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="var" value="main"/>
  <xacro:include filename="include1.xacro" ns="A"/>
  <xacro:include filename="include2.xacro" ns="B"/>
  <A.foo/><B.foo/>
  <main var="${var}" A="${2*A.var}" B="${B.var+1}"/>
</a>'''
        result = '''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
    <inc1/><inc2/><main var="main" A="2" B="3"/>
</a>'''

        if self.in_order:
            self.assert_matches(self.quick_xacro(doc), result)
        else:
            self.assertRaises(xacro.XacroException, self.quick_xacro, doc)

    def test_boolean_if_statement(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="false">
    <a />
  </xacro:if>
  <xacro:if value="true">
    <b />
  </xacro:if>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <b />
</robot>''')

    def test_invalid_if_statement(self):
        self.assertRaises(xacro.XacroException,
                          self.quick_xacro,
                          '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                          <xacro:if value="nonsense"><foo/></xacro:if></a>''')

    def test_integer_if_statement(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${0*42}">
    <a />
  </xacro:if>
  <xacro:if value="0">
    <b />
  </xacro:if>
  <xacro:if value="${0}">
    <c />
  </xacro:if>
  <xacro:if value="${1*2+3}">
    <d />
  </xacro:if>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <d />
</robot>''')

    def test_float_if_statement(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${3*0.0}">
    <a />
  </xacro:if>
  <xacro:if value="${3*0.1}">
    <b />
  </xacro:if>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <b />
</robot>''')

    def test_boolean_if_statement(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="condT" value="${True}"/>
  <xacro:property name="condF" value="${False}"/>
  <xacro:if value="${condF}"><a /></xacro:if>
  <xacro:if value="${condT}"><b /></xacro:if>
  <xacro:if value="${True}"><c /></xacro:if>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <b /><c />
</robot>''')

    def test_consecutive_if(self):
        self.assert_matches(self.quick_xacro('''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><xacro:if value="0"><a>bar</a></xacro:if></xacro:if>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro"/>''')

    def test_equality_expression_in_if_statement(self):
        self.assert_matches(self.quick_xacro('''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="var" value="useit"/>
  <xacro:if value="${var == 'useit'}"><foo>bar</foo></xacro:if>
  <xacro:if value="${'use' in var}"><bar>foo</bar></xacro:if>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<foo>bar</foo>
<bar>foo</bar>
</a>''')

    def test_no_evaluation(self):
        self.assert_matches(self.quick_xacro('''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="xyz" value="5 -2"/>
  <foo>${xyz}</foo>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <foo>5 -2</foo>
</a>''')

    def test_math_expressions(self):
        self.assert_matches(self.quick_xacro('''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <foo function="${1. + sin(pi)}"/>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <foo function="1.0"/>
</a>''')

    def test_consider_non_elements_if(self):
        self.assert_matches(self.quick_xacro('''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><!-- comment --> text <b>bar</b></xacro:if>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<!-- comment --> text <b>bar</b></a>''')

    def test_consider_non_elements_block(self):
        self.assert_matches(
                self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="foo" params="*block">
  <!-- comment -->
  foo
  <xacro:insert_block name="block" />
</xacro:macro>
<xacro:foo>
  <!-- ignored comment -->
  ignored text
  <a_block />
</xacro:foo>
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- comment -->
  foo
  <a_block />
</a>''')

    def test_ignore_xacro_comments(self):
        self.assert_matches(self.quick_xacro('''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- A -->

  <!-- ignore multiline comments before any xacro tag -->
  <!-- ignored -->
  <xacro:property name="foo" value="1"/>
  <!-- ignored -->
  <xacro:if value="1"><!-- B --></xacro:if>
  <!-- ignored -->
  <xacro:macro name="foo"><!-- C --></xacro:macro>
  <!-- ignored -->
  <xacro:foo/>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<!-- A --><!-- B --><!-- C --></a>''')

    def test_recursive_evaluation(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value=" 42 "/>
  <xacro:property name="a2" value="${ 2 * a }"/>
  <a doubled="${a2}"/>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a doubled="84"/>
</robot>''')

    def test_recursive_evaluation_wrong_order(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a2" value="${2*a}"/>
  <xacro:property name="a" value="42"/>
  <a doubled="${a2}"/>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a doubled="84"/>
</robot>''')

    def test_recursive_definition(self):
        self.assertRaises(xacro.XacroException,
                          self.quick_xacro, '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="${a2}"/>
  <xacro:property name="a2" value="${2*a}"/>
  <a doubled="${a2}"/>
</robot>''')

    def test_multiple_recursive_evaluation(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="1"/>
  <xacro:property name="b" value="2"/>
  <xacro:property name="c" value="3"/>
  <xacro:property name="product" value="${a*b*c}"/>
  <answer product="${product}"/>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <answer product="6"/>
</robot>''')

    def test_multiple_definition_and_evaluation(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="42"/>
  <xacro:property name="b" value="${a}"/>
  <xacro:property name="b" value="${-a}"/>
  <xacro:property name="b" value="${a}"/>
  <answer b="${b} ${b} ${b}"/>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <answer b="42 42 42"/>
</robot>''')

    def test_transitive_evaluation(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="42"/>
  <xacro:property name="b" value="${a}"/>
  <xacro:property name="c" value="${b}"/>
  <xacro:property name="d" value="${c}"/>
  <answer d="${d}"/>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <answer d="42"/>
</robot>''')

    def test_multi_tree_evaluation(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="42"/>
  <xacro:property name="b" value="2.1"/>
  <xacro:property name="c" value="${a}"/>
  <xacro:property name="d" value="${b}"/>
  <xacro:property name="f" value="${c*d}"/>
  <answer f="${f}"/>
</robot>'''), 
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <answer f="88.2"/>
</robot>''')

    def test_from_issue(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="42"/>
  <xacro:property name="wheel_width" value="${x}"/>
  <link name="my_link">
    <origin xyz="0 0 ${wheel_width/2}"/>
  </link>
</robot>'''), 
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="my_link">
    <origin xyz="0 0 21.0"/>
  </link>
</robot>''')

    def test_recursive_bad_math(self):
        self.assertRaises(xacro.XacroException,
            self.quick_xacro, '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="0"/>
  <tag badness="${1/x}"/>
</robot>''')

    def test_default_param(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="fixed_link" params="parent_link:=base_link child_link *joint_pose">
    <link name="${child_link}"/>
    <joint name="${child_link}_joint" type="fixed">
      <xacro:insert_block name="joint_pose" />
      <parent link="${parent_link}"/>
      <child link="${child_link}" />
    </joint>
  </xacro:macro>
  <xacro:fixed_link child_link="foo">
    <origin xyz="0 0 0" rpy="0 0 0" />
  </xacro:fixed_link >
</robot>'''), 
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="foo"/>
  <joint name="foo_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="foo"/>
  </joint>
</robot>''')

    def test_default_param_override(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="fixed_link" params="parent_link:=base_link child_link *joint_pose">
    <link name="${child_link}"/>
    <joint name="${child_link}_joint" type="fixed">
      <xacro:insert_block name="joint_pose" />
      <parent link="${parent_link}"/>
      <child link="${child_link}" />
    </joint>
  </xacro:macro>
  <xacro:fixed_link child_link="foo" parent_link="bar">
    <origin xyz="0 0 0" rpy="0 0 0" />
  </xacro:fixed_link >
</robot>'''), 
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="foo"/>
  <joint name="foo_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="bar"/>
    <child link="foo"/>
  </joint>
</robot>''')

    def test_param_missing(self):
        self.assertRaises(xacro.XacroException,
                          self.quick_xacro, '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="fixed_link" params="parent_link child_link *joint_pose">
    <link name="${child_link}"/>
    <joint name="${child_link}_joint" type="fixed">
      <xacro:insert_block name="joint_pose" />
      <parent link="${parent_link}"/>
      <child link="${child_link}" />
    </joint>
  </xacro:macro>
  <xacro:fixed_link child_link="foo">
    <origin xyz="0 0 0" rpy="0 0 0" />
  </xacro:fixed_link >
</robot>''')

    def test_default_arg(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="foo" default="2"/>
  <link name="my_link">
    <origin xyz="0 0 $(arg foo)"/>
  </link>
</robot>
'''),'''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="my_link">
    <origin xyz="0 0 2"/>
  </link>
</robot>''')

    def test_default_arg_override(self):
        self.assert_matches(
                self.quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="foo" default="2"/>
  <link name="my_link">
    <origin xyz="0 0 $(arg foo)"/>
  </link>
</robot>
''', ['foo:=4']),'''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="my_link">
    <origin xyz="0 0 4"/>
  </link>
</robot>''')

    def test_default_arg_missing(self):
        self.assertRaises(Exception,
            self.quick_xacro, '''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a arg="$(arg foo)"/>
</a>
''')

    def test_default_arg_empty(self):
        self.assert_matches(self.quick_xacro('''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:arg name="foo" default=""/>$(arg foo)</a>'''),
                            '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"/>''')

    def test_broken_input_doesnt_create_empty_output_file(self):
        # run xacro on broken input file to make sure we don't create an
        # empty output file
        tmp_dir_name = tempfile.mkdtemp() # create directory we can trash
        output_path = os.path.join(tmp_dir_name, "should_not_exist")
        self.run_xacro('broken.xacro', '-o', output_path)

        output_file_created = os.path.isfile(output_path)
        shutil.rmtree(tmp_dir_name) # clean up after ourselves

        self.assertFalse(output_file_created)

    def test_create_subdirs(self):
        # run xacro to create output file in non-existent directory
        # to make sure this directory will be created by xacro
        tmp_dir_name = tempfile.mkdtemp() # create directory we can trash
        shutil.rmtree(tmp_dir_name) # ensure directory is removed
        output_path = os.path.join(tmp_dir_name, "out")
        self.run_xacro('include1.xml', '-o', output_path)

        output_file_created = os.path.isfile(output_path)
        shutil.rmtree(tmp_dir_name) # clean up after ourselves

        self.assertTrue(output_file_created)

    def test_iterable_literals_plain(self):
        self.assert_matches(
                self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="list" value="[0, 1+1, 2]"/>
  <xacro:property name="tuple" value="(0,1+1,2)"/>
  <xacro:property name="dict" value="{'a':0, 'b':1+1, 'c':2}"/>
  <a list="${list}" tuple="${tuple}" dict="${dict}"/>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a list="[0, 1+1, 2]" tuple="(0,1+1,2)" dict="{'a':0, 'b':1+1, 'c':2}"/>
</a>''')

    def test_iterable_literals_eval(self):
        self.assert_matches(
                self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="list" value="${[0, 1+1, 2]}"/>
  <xacro:property name="tuple" value="${(0,1+1,2)}"/>
  <xacro:property name="dic" value="${dict(a=0, b=1+1, c=2)}"/>
  <a list="${list}" tuple="${tuple}" dict="${dic}"/>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a list="[0, 2, 2]" tuple="(0, 2, 2)" dict="{'a': 0, 'c': 2, 'b': 2}"/>
</a>''')

    def test_enforce_xacro_ns(self):
        self.assert_matches(
                self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <arg name="foo" value="bar"/>
  <include filename="foo"/>
</a>''', xacro_ns=False),
'''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <arg name="foo" value="bar"/>
  <include filename="foo"/>
</a>''')

    def test_issue_68_numeric_arg(self):
        # If a property is assigned from a substitution arg, then this properties' value was
        # no longer converted to a python type, so that e.g. 0.5 remained u'0.5'.
        # If this property is then used in a numerical expression an exception is thrown.
        self.assert_matches(
                self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="foo" default="0.5"/>
  <xacro:property name="prop" value="$(arg foo)" />
  <a prop="${prop-0.3}"/>
</a>
'''),'''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a prop="0.2"/>
</a>''')

    def test_transitive_arg_evaluation(self):
        self.assert_matches(
                self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="foo" default="0.5"/>
  <xacro:arg name="bar" default="$(arg foo)"/>
  <xacro:property name="prop" value="$(arg bar)" />
  <a prop="${prop-0.3}"/>
</a>
'''),'''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a prop="0.2"/>
</a>''')

    def test_macro_name_with_colon(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:macro name="xacro:my_macro"><foo/></xacro:macro>
        <xacro:my_macro/>
        </a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><foo/></a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_overwrite_globals(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:property name="pi"  value="3.14"/></a>'''
        with capture_stderr(self.quick_xacro, src) as (result, output):
            self.assert_matches(result, '<a xmlns:xacro="http://www.ros.org/wiki/xacro"/>')
            self.assertTrue(output)

    def test_no_double_evaluation(self):
        src = '''
<a xmlns:xacro="http://www.ros.org/xacro">
  <xacro:macro name="foo" params="a b:=${a} c:=$${a}"> a=${a} b=${b} c=${c} </xacro:macro>
  <xacro:property name="a" value="1"/>
  <xacro:property name="d" value="$${a}"/>
  <d d="${d}"><foo a="2"/></d>
</a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/xacro"><d d="${a}"> a=2 b=1 c=${a} </d></a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_property_forwarding(self):
        src='''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:property name="arg" value="42"/>
        <xacro:macro name="foo" params="arg:=^%s">${arg}</xacro:macro>
        <xacro:foo/>
        </a>'''
        res='''<a xmlns:xacro="http://www.ros.org/wiki/xacro">%s</a>'''
        self.assert_matches(self.quick_xacro(src % ''), res % '42')
        self.assert_matches(self.quick_xacro(src % '|'), res % '42')
        self.assert_matches(self.quick_xacro(src % '|6'), res % '42')

    def test_extension_in_expression(self):
        src='''<a xmlns:xacro="http://www.ros.org/wiki/xacro">${2*'$(arg var)'}</a>'''
        res='''<a xmlns:xacro="http://www.ros.org/wiki/xacro">%s</a>'''
        self.assert_matches(self.quick_xacro(src, ['var:=xacro']), res % (2*'xacro'))

    def test_expression_in_extension(self):
        src='''<a xmlns:xacro="http://www.ros.org/wiki/xacro">$(arg ${'v'+'ar'})</a>'''
        res='''<a xmlns:xacro="http://www.ros.org/wiki/xacro">%s</a>'''
        self.assert_matches(self.quick_xacro(src, ['var:=xacro']), res % 'xacro')

# test class for in-order processing
class TestXacroInorder(TestXacro):
    def __init__(self, *args, **kwargs):
        super(TestXacroInorder, self).__init__(*args, **kwargs)
        self.in_order = True

    def test_include_lazy(self):
        doc = ('''<a xmlns:xacro="http://www.ros.org/xacro">
        <xacro:if value="false"><xacro:include filename="non-existent"/></xacro:if></a>''')
        self.assert_matches(self.quick_xacro(doc),
                        '''<a xmlns:xacro="http://www.ros.org/xacro"/>''')

    def test_issue_63_fixed_with_inorder_processing(self):
        self.assert_matches(
                self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="has_stuff" default="false"/>
  <xacro:if value="$(arg has_stuff)">
    <xacro:include file="$(find nonexistent_package)/stuff.urdf" />
  </xacro:if>
</a>'''),
'<a xmlns:xacro="http://www.ros.org/wiki/xacro"/>')

    def test_yaml_support(self):
        src = '''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="settings" value="${load_yaml('settings.yaml')}"/>
  <xacro:property name="type" value="$(arg type)"/>
  <xacro:include filename="${settings['arms'][type]['file']}"/>
  <xacro:call macro="${settings['arms'][type]['macro']}"/>
</a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><{tag}/></a>'''
        for i in ['inc1', 'inc2']:
            self.assert_matches(self.quick_xacro(src, cli=['type:=%s' % i]),
                                res.format(tag=i))

    def test_macro_default_param_evaluation_order(self):
        src='''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="foo" params="arg:=${2*foo}">
    <xacro:property name="foo" value="-"/>
    <f val="${arg}"/>
</xacro:macro>
<xacro:property name="foo" value="${3*7}"/>
<xacro:foo/>
<xacro:property name="foo" value="*"/>
<xacro:foo/>
</a>'''
        res='''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<f val="42"/><f val="**"/></a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_check_order_warning(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:property name="bar" value="unused"/>
<xacro:property name="foo" value="unused"/>
<xacro:macro name="foo" params="arg:=${foo}">
    <a val="${arg}"/>
</xacro:macro>
<xacro:foo/>
<xacro:property name="bar" value="dummy"/>
<xacro:property name="foo" value="21"/></a>'''
        with capture_stderr(self.quick_xacro, src, do_check_order=True) as (result, output):
            self.assertTrue("Document is incompatible to in-order processing." in output)
            self.assertTrue("foo" in output)  # foo should be reported
            self.assertTrue("bar" not in output)  # bar shouldn't be reported

    def test_default_property(self):
        src = '''
        <a xmlns:xacro="http://www.ros.org/xacro">
            <xacro:property name="prop" default="false"/>
            <xacro:unless value="${prop}">
                <foo/>
                <xacro:property name="prop" value="true"/>
            </xacro:unless>

            <!-- second foo should be ignored -->
            <xacro:unless value="${prop}">
                <foo/>
                <xacro:property name="prop" value="true"/>
            </xacro:unless>
        </a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/xacro"><foo/></a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_unicode_literal_parsing(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">🍔 </a>'''
        self.assert_matches(self.quick_xacro(src), src)

    def test_unicode_property(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:property name="burger" value="🍔"/>
${burger}</a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">🍔</a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_unicode_property_attribute(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:property name="burger" value="🍔"/>
<b c="${burger}"/></a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro"><b c="🍔"/></a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_unicode_property_block(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:property name="burger">
🍔
</xacro:property>
<xacro:insert_block name="burger"/></a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">🍔</a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_unicode_conditional(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:property name="burger" value="🍔"/>
<xacro:if value="${burger == u'🍔'}">
🍟
</xacro:if>
</a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">🍟</a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_unicode_macro(self):
        src = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="burger" params="how_many">
${u'🍔' * how_many}
</xacro:macro>
<xacro:burger how_many="4"/>
</a>'''
        res = '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
🍔🍔🍔🍔</a>'''
        self.assert_matches(self.quick_xacro(src), res)

    def test_unicode_file(self):
        # run the full xacro processing pipeline on a file with
        # unicode characters in it and make sure the output is correct
        test_dir= os.path.abspath(os.path.dirname(__file__))
        input_path = os.path.join(test_dir, 'emoji.xacro')
        tmp_dir_name = tempfile.mkdtemp() # create directory we can trash
        output_path = os.path.join(tmp_dir_name, "out.xml")
        self.run_xacro(input_path, '-o', output_path)
        output_file_created = os.path.isfile(output_path)
        self.assert_matches(xml.dom.minidom.parse(output_path),
            '''<robot xmlns:xacro="http://ros.org/wiki/xacro">🍔</robot>''')
        shutil.rmtree(tmp_dir_name) # clean up after ourselves

if __name__ == '__main__':
    unittest.main()
