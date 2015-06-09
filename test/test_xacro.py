#! /usr/bin/env python

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
from cStringIO import StringIO
from contextlib import contextmanager


# regex to match whitespace
whitespace = re.compile(r'\s+')

def all_attributes_match(a, b):
    if len(a.attributes) != len(b.attributes):
        print("Different number of attributes")
        return False
    a_atts = [(a.attributes.item(i).name, a.attributes.item(i).value) for i in range(len(a.attributes))]
    b_atts = [(b.attributes.item(i).name, b.attributes.item(i).value) for i in range(len(b.attributes))]
    a_atts.sort()
    b_atts.sort()

    for i in range(len(a_atts)):
        if a_atts[i][0] != b_atts[i][0]:
            print("Different attribute names: %s and %s" % (a_atts[i][0], b_atts[i][0]))
            return False
        try:
            if abs(float(a_atts[i][1]) - float(b_atts[i][1])) > 1.0e-9:
                print("Different attribute values: %s and %s" % (a_atts[i][1], b_atts[i][1]))
                return False
        except ValueError:  # Attribute values aren't numeric
            if a_atts[i][1] != b_atts[i][1]:
                print("Different attribute values: %s and %s" % (a_atts[i][1], b_atts[i][1]))
                return False

    return True

def text_matches(a, b):
    a_norm = whitespace.sub(' ', a)
    b_norm = whitespace.sub(' ', b)
    if a_norm.strip() == b_norm.strip(): return True
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

    def test_empty_node_vs_whitespace(self):
        self.assertTrue(xml_matches('''<foo/>''', '''<foo> \t\n\r </foo>'''))
    def test_whitespace_vs_empty_node(self):
        self.assertTrue(xml_matches('''<foo> \t\n\r </foo>''', '''<foo/>'''))
    def test_normalize_whitespace_nested(self):
        self.assertTrue(xml_matches('''<a><b/></a>''', '''<a>\n<b> </b> </a>'''))

    def test_ignore_comments(self):
        self.assertTrue(xml_matches('''<a><b/><!-- foo --> <!-- bar --></a>''',
                                    '''<a><b/></a>''', [xml.dom.Node.COMMENT_NODE]))


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
            opts, _ = xacro.process_cli_args(cli, require_input=False)
            args.update(vars(opts))  # initialize with cli args
        args.update(dict(in_order = self.in_order))  # set in_order option from test class
        args.update(kwargs)  # explicit function args have highest priority

        doc = xacro.parse(xml)
        xacro.process_doc(doc, **args)
        return doc

    def run_xacro(self, input_path, *args):
        args = list(args)
        if self.in_order:
            args.append('--inorder')
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

    def test_macro_undefined(self):
        self.assertRaises(xacro.XacroException,
                          self.quick_xacro,
                          '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
                          <xacro:undefined><foo/><bar/></xacro:undefined></a>''')

    def test_inorder_processing(self):
        src = '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="1.0"/>
  <xacro:property name="mount" value="base1"/>
  <xacro:macro name="ee" params="side *origin">
    <link name="${side}_base1"> <xacro:insert_block name="origin"/> </link>
  </xacro:macro>
  <xacro:ee side="left"> <origin>1 ${foo}</origin> </xacro:ee>
  <joint name="mount" type="fixed"> <child link="${mount}"/> </joint>

  <xacro:property name="foo" value="3.0"/>
  <xacro:property name="mount" value="base2"/>
  <xacro:macro name="ee" params="side *origin">
    <link name="${side}_base2"> <xacro:insert_block name="origin"/> </link>
  </xacro:macro>
  <xacro:ee side="right"> <origin>2 ${foo}</origin> </xacro:ee>
  <joint name="mount" type="fixed"> <child link="${mount}"/> </joint>
</robot>'''
        oldOrder = '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="left_base2"> <origin>1 3.0</origin> </link>
  <joint name="mount" type="fixed"> <child link="base2"/> </joint>

  <link name="right_base2"> <origin>2 3.0</origin> </link>
  <joint name="mount" type="fixed"> <child link="base2"/> </joint>
</robot>'''
        inOrder = '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="left_base1"> <origin>1 1.0</origin> </link>
  <joint name="mount" type="fixed"> <child link="base1"/> </joint>

  <link name="right_base2"> <origin>2 3.0</origin> </link>
  <joint name="mount" type="fixed"> <child link="base2"/> </joint>
</robot>'''
        self.assert_matches(self.quick_xacro(src), inOrder if self.in_order else oldOrder)

    def test_DEPRECATED_should_replace_before_macroexpand(self):
        self.assert_matches(
                self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="inner" params="*the_block">
  <in_the_inner><xacro:insert_block name="the_block" /></in_the_inner>
</xacro:macro>
<xacro:macro name="outer" params="*the_block">
  <in_the_outer><inner><xacro:insert_block name="the_block" /></inner></in_the_outer>
</xacro:macro>
<outer><woot /></outer></a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<in_the_outer><in_the_inner><woot /></in_the_inner></in_the_outer></a>''')

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

    def test_property_replacement(self):
        self.assert_matches(
                self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="42" />
  <the_foo result="${foo}" />
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <the_foo result="42" />
</a>''')

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
                self.quick_xacro('''<a b="$${foo}" c="$$${foo}" />'''),
                '''<a b="${foo}" c="$${foo}" />''')

    def test_just_a_dollar_sign(self):
        self.assert_matches(
                self.quick_xacro('''<a b="$" />'''),
                '''<a b="$" />''')

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
        self.assert_matches(
                self.quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="foo" params="*block1 *block2">
  <xacro:insert_block name="block2" />
  <first>
    <xacro:insert_block name="block1" />
  </first>
</xacro:macro>
<xacro:foo>
  <first_block />
  <second_block />
</xacro:foo>
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <second_block />
  <first>
    <first_block />
  </first>
</a>''')

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
<xacro:property name="some_block">
  <some_block />
</xacro:property>
<foo>
  <xacro:insert_block name="some_block" />
</foo>
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<foo><some_block /></foo>
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
        self.assertRaises(IOError,
                          self.quick_xacro, '''<a xmlns:xacro="http://www.ros.org/xacro">
                             <xacro:include filename="include-nada.xml" /></a>''')

    def test_include_lazy(self):
        doc = ('''<a xmlns:xacro="http://www.ros.org/xacro">
        <xacro:if value="false"><xacro:include filename="non-existent"/></xacro:if></a>''')
        self.assert_matches(self.quick_xacro(doc, in_order=True),
                        '''<a xmlns:xacro="http://www.ros.org/xacro"/>''')

    def test_include_from_variable(self):
        doc = ('''<a xmlns:xacro="http://www.ros.org/xacro">
        <xacro:property name="file" value="include1.xml"/>
        <xacro:include filename="${file}" /></a>''')
        self.assert_matches(self.quick_xacro(doc, in_order=True),
                        '''<a xmlns:xacro="http://www.ros.org/xacro"><inc1/></a>''')

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
  <xacro:property name="e" value="${c*d}"/>
  <answer e="${e}"/>
</robot>'''), 
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <answer e="88.2"/>
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

    def test_ros_arg_param(self):
        self.assert_matches(
                self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <arg name="foo" value="bar"/>
</a>''', xacro_ns=False),
'''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <arg name="foo" value="bar"/>
</a>''')

    def test_issue_63_fixed_with_inorder_processing(self):
        self.assert_matches(
                self.quick_xacro('''\
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="has_stuff" default="false"/>
  <xacro:if value="$(arg has_stuff)">
    <xacro:include file="$(find nonexistent_package)/stuff.urdf" />
  </xacro:if>
</a>''', in_order=True),
'<a xmlns:xacro="http://www.ros.org/wiki/xacro"/>')

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


# test class for in-order processing
class TestXacroInorder(TestXacro):
    def __init__(self, *args, **kwargs):
        super(TestXacroInorder, self).__init__(*args, **kwargs)
        self.in_order = True


if __name__ == '__main__':
    unittest.main()
