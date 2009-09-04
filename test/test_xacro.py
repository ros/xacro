#! /usr/bin/env python
import roslib
roslib.load_manifest('xacro')

import sys
import unittest
import xacro
from xml.dom.minidom import parse, parseString
import xml.dom

def all_attributes_match(a, b):
    if len(a.attributes) != len(b.attributes):
        print "Different number of attributes"
        return False
    a_atts = [(a.attributes.item(i).name, a.attributes.item(i).value) for i in range(len(a.attributes))]
    b_atts = [(b.attributes.item(i).name, b.attributes.item(i).value) for i in range(len(b.attributes))]
    a_atts.sort()
    b_atts.sort()

    for i in range(len(a_atts)):
        if a_atts[i][0] != b_atts[i][0]:
            print "Different attribute names: %s and %s" % (a_atts[i][0], b_atts[i][0])
            return False
        try:
            if abs(float(a_atts[i][1]) - float(b_atts[i][1])) > 1.0e-9:
                print "Different attribute values: %s and %s" % (a_atts[i][1], b_atts[i][1])
                return False
        except ValueError: # Attribute values aren't numeric
            if a_atts[i][1] != b_atts[i][1]:
                print "Different attribute values: %s and %s" % (a_atts[i][1], b_atts[i][1])
                return False

    return True

def elements_match(a, b):
    if not a and not b:
        return True
    if not a or not b:
        return False

    if a.nodeType != b.nodeType:
        print "Different node types: %d and %d" % (a.nodeType, b.nodeType)
        return False
    if a.nodeName != b.nodeName:
        print "Different element names: %s and %s" % (a.nodeName, b.nodeName)
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
        print "Match failed:"
        a.writexml(sys.stdout)
        print
        print '='*78
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

    def test_should_replace_before_macroexpand(self):
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

    def test_math_ignores_spaces(self):
        self.assertTrue(
            xml_matches(
                quick_xacro('''<a><f v="${0.9 / 2 - 0.2}" /></a>'''),
                '''<a><f v="0.25" /></a>'''))


if __name__ == '__main__':
    import rostest
    rostest.unitrun('xacro', 'test_xacro', TestXacro)
