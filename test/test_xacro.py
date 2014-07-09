#! /usr/bin/env python

from __future__ import print_function

import sys
import unittest
import xacro
from xml.dom.minidom import parseString
import xml.dom
import os.path
from rosgraph.names import load_mappings
from xacro import set_substitution_args_context


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


def elements_match(a, b):
    if not a and not b:
        return True
    if not a or not b:
        return False

    if a.nodeType != b.nodeType:
        print("Different node types: %d and %d" % (a.nodeType, b.nodeType))
        return False
    if a.nodeName != b.nodeName:
        print("Different element names: %s and %s" % (a.nodeName, b.nodeName))
        return False

    if not all_attributes_match(a, b):
        return False

    if not elements_match(xacro.first_child_element(a), xacro.first_child_element(b)):
        return False
    if not elements_match(xacro.next_sibling_element(a), xacro.next_sibling_element(b)):
        return False
    return True


def xml_matches(a, b):
    if isinstance(a, str):
        return xml_matches(parseString(a).documentElement, b)
    if isinstance(b, str):
        return xml_matches(a, parseString(b).documentElement)
    if a.nodeType == xml.dom.Node.DOCUMENT_NODE:
        return xml_matches(a.documentElement, b)
    if b.nodeType == xml.dom.Node.DOCUMENT_NODE:
        return xml_matches(a, b.documentElement)

    if not elements_match(a, b):
        print("Match failed:")
        a.writexml(sys.stdout)
        print
        print('=' * 78)
        b.writexml(sys.stdout)
        return False
    return True


def quick_xacro(xml):
    if isinstance(xml, str):
        doc = parseString(xml)
        return quick_xacro(doc)
    xacro.eval_self_contained(xml)
    return xml


class TestXacro(unittest.TestCase):

    def test_DEPRECATED_should_replace_before_macroexpand(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a>
<macro name="inner" params="*the_block">
  <in_the_inner><insert_block name="the_block" /></in_the_inner>
</macro>
<macro name="outer" params="*the_block">
  <in_the_outer><inner><insert_block name="the_block" /></inner></in_the_outer>
</macro>
<outer><woot /></outer></a>'''),
                '''<a>
<in_the_outer><in_the_inner><woot /></in_the_inner></in_the_outer></a>'''))

    def test_should_replace_before_macroexpand(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="inner" params="*the_block">
  <in_the_inner><xacro:insert_block name="the_block" /></in_the_inner>
</xacro:macro>
<xacro:macro name="outer" params="*the_block">
  <in_the_outer><xacro:inner><insert_block name="the_block" /></xacro:inner></in_the_outer>
</xacro:macro>
<xacro:outer><woot /></xacro:outer></a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<in_the_outer><in_the_inner><woot /></in_the_inner></in_the_outer></a>'''))

    def test_property_replacement(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="42" />
  <the_foo result="${foo}" />
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <the_foo result="42" />
</a>'''))

    def test_math_ignores_spaces(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a><f v="${0.9 / 2 - 0.2}" /></a>'''),
                '''<a><f v="0.25" /></a>'''))

    def test_substitution_args_find(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a><f v="$(find xacro)/test/test_xacro.py" /></a>'''),
                '''<a><f v="''' + os.path.abspath((__file__).replace(".pyc",".py")) + '''" /></a>'''))

    def test_substitution_args_arg(self):
        set_substitution_args_context(load_mappings(['sub_arg:=my_arg']))
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a><f v="$(arg sub_arg)" /></a>'''),
                '''<a><f v="my_arg" /></a>'''))
        set_substitution_args_context({})

    def test_escaping_dollar_braces(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a b="$${foo}" c="$$${foo}" />'''),
                '''<a b="${foo}" c="$${foo}" />'''))

    def test_just_a_dollar_sign(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a b="$" />'''),
                '''<a b="$" />'''))

    def test_multiple_insert_blocks(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
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
</a>'''))

    def test_multiple_blocks(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
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
</a>'''))

    def test_integer_stays_integer(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:macro name="m" params="num">
  <test number="${num}" />
</xacro:macro>
<xacro:m num="100" />
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <test number="100" />
</a>'''))

    def test_insert_block_property(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:property name="some_block">
  <some_block />
</xacro:property>
<foo>
  <xacro:insert_block name="some_block" />
</foo>
</a>'''),
                '''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<foo><some_block /></foo>
</a>'''))

    def test_include(self):
        doc = parseString('''<a xmlns:xacro="http://www.ros.org/xacro">
                             <xacro:include filename="include1.xml" /></a>''')
        xacro.process_includes(doc, os.path.dirname(os.path.realpath(__file__)))
        self.assertTrue(
            xml_matches(doc, '''<a xmlns:xacro="http://www.ros.org/xacro"><foo /><bar /></a>'''))

    def test_include_glob(self):
        doc = parseString('''<a xmlns:xacro="http://www.ros.org/xacro">
                             <xacro:include filename="include*.xml" /></a>''')
        xacro.process_includes(doc, os.path.dirname(os.path.realpath(__file__)))
        self.assertTrue(
            xml_matches(doc, '<a xmlns:xacro="http://www.ros.org/xacro"><foo /><bar /><baz /></a>'))

    def test_include_nonexistent(self):
        doc = parseString('''<a xmlns:xacro="http://www.ros.org/xacro">
                             <xacro:include filename="include-nada.xml" /></a>''')
        self.assertRaises(xacro.XacroException,
                          xacro.process_includes, doc, os.path.dirname(os.path.realpath(__file__)))

    def test_boolean_if_statement(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))      

    def test_integer_if_statement(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))      

    def test_float_if_statement(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))

    def test_recursive_evaluation(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="42"/>
  <xacro:property name="a2" value="${2*a}"/>
  <a doubled="${a2}"/>
</robot>'''),
                '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <a doubled="84"/>
</robot>'''))

    def test_recursive_definition(self):
        self.assertRaises(xacro.XacroException,
                          quick_xacro, '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="${a2}"/>
  <xacro:property name="a2" value="${2*a}"/>
  <a doubled="${a2}"/>
</robot>''')

    def test_multiple_recursive_evaluation(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))

    def test_transitive_evaluation(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))

    def test_multi_tree_evaluation(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))

    def test_from_issue(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))

    def test_recursive_bad_math(self):
        self.assertRaises(ZeroDivisionError,
            quick_xacro, '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="0"/>
  <tag badness="${1/x}"/>
</robot>''')

