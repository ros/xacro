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
from rosgraph.names import load_mappings
from xacro import set_substitution_args_context

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

def nodes_match(a, b):
    ignore = [] # list of node type to ignore
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
               ((a.nodeType in ignore) or
                (a.nodeType == xml.dom.Node.TEXT_NODE and whitespace.sub('', a.data) == ""))):
            a = a.nextSibling
        while (b and
               ((b.nodeType in ignore) or
                (b.nodeType == xml.dom.Node.TEXT_NODE and whitespace.sub('', b.data) == ""))):
            b = b.nextSibling

        if not nodes_match(a, b):
            return False

        if a: a = a.nextSibling
        if b: b = b.nextSibling

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

    if not nodes_match(a, b):
        print("Match failed:")
        a.writexml(sys.stdout)
        print()
        print('=' * 78)
        b.writexml(sys.stdout)
        print()
        return False
    return True


def quick_xacro(xml):
    if isinstance(xml, str):
        doc = parseString(xml)
        return quick_xacro(doc)
    xacro.eval_self_contained(xml)
    return xml


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

    def test_consecutive_if(self):
        self.assertTrue(
            xml_matches(quick_xacro('''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><xacro:if value="0"><a>bar</a></xacro:if></xacro:if>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro"/>'''))

    def test_consider_non_elements_if(self):
        self.assertTrue(
            xml_matches(quick_xacro('''
<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><!-- comment --> text <b>bar</b></xacro:if>
</a>'''),
'''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
<!-- comment --> text <b>bar</b></a>'''))

    def test_consider_non_elements_block(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a xmlns:xacro="http://www.ros.org/wiki/xacro">
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
</a>'''))

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

    def test_pr2(self):
        # run xacro on the pr2 tree snapshot
        test_dir= os.path.abspath(os.path.dirname(__file__))
        xacro_path = os.path.join(test_dir, '..', 'xacro.py')
        pr2_xacro_path = os.path.join(test_dir, 'robots', 'pr2',
                                      'pr2.urdf.xacro')
        proc = subprocess.Popen([xacro_path, pr2_xacro_path],
                                stdout=subprocess.PIPE)
        output, errcode = proc.communicate()
        if errcode:
            raise Exception("xacro couldn't process the pr2 snapshot test case")
        pr2_golden_parse_path = os.path.join(test_dir, 'robots', 'pr2',
                                             'pr2_1.11.4.xml')
        self.assertTrue(
            xml_matches(
                xml.dom.minidom.parse(pr2_golden_parse_path),
                quick_xacro(output)))

    def test_default_param(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))

    def test_default_param_override(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))

    def test_param_missing(self):
        self.assertRaises(xacro.XacroException,
                          quick_xacro, '''\
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
        set_substitution_args_context({})
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
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
</robot>'''))
        set_substitution_args_context({})

    def test_default_arg_override(self):
        set_substitution_args_context(load_mappings(['foo:=4']))
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="foo" default="2"/>
  <link name="my_link">
    <origin xyz="0 0 $(arg foo)"/>
  </link>
</robot>
'''),'''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="my_link">
    <origin xyz="0 0 4"/>
  </link>
</robot>'''))
        set_substitution_args_context({})

    def test_default_arg_missing(self):
        set_substitution_args_context({})
        self.assertRaises(Exception,
            quick_xacro, '''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="my_link">
    <origin xyz="0 0 $(arg foo)"/>
  </link>
</robot>
''')
        set_substitution_args_context({})

    def test_broken_input_doesnt_create_empty_output_file(self):
        # run xacro on broken input file to make sure we don't create an
        # empty output file
        tmp_dir_name = tempfile.mkdtemp() # create directory we can trash
        test_dir = os.path.abspath(os.path.dirname(__file__))
        output_path = os.path.join(tmp_dir_name, "should_not_exist")
        xacro_path = os.path.join(test_dir, '..', 'xacro.py')
        broken_file_path = os.path.join(test_dir, 'broken.xacro')
        errcode = subprocess.call([xacro_path, broken_file_path,
                                   '-o', output_path])
        output_file_created = os.path.isfile(output_path)
        shutil.rmtree(tmp_dir_name) # clean up after ourselves
        self.assertFalse(output_file_created)

    def test_ros_arg_param(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''\
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="fixed_link" params="parent_link:=base_link child_link *joint_pose">
    <link name="${child_link}"/>
    <joint name="${child_link}_joint" type="fixed">
      <xacro:insert_block name="joint_pose" />
      <parent link="${parent_link}"/>
      <child link="${child_link}" />
      <arg name="${parent_link}" value="${child_link}"/>
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
    <arg name="base_link" value="foo"/>
  </joint>
</robot>'''))

