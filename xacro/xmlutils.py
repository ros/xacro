#!/usr/bin/env python3

# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright (c) 2015, Open Source Robotics Foundation, Inc.
# Copyright (c) 2013, Willow Garage, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Stuart Glaser, William Woodall, Robert Haschke
# Maintainer: Morgan Quigley <morgan@osrfoundation.org>

import xml.dom.minidom

from .color import warning


def first_child_element(elt):
    c = elt.firstChild
    while c and c.nodeType != xml.dom.Node.ELEMENT_NODE:
        c = c.nextSibling
    return c


def next_sibling_element(node):
    c = node.nextSibling
    while c and c.nodeType != xml.dom.Node.ELEMENT_NODE:
        c = c.nextSibling
    return c


def replace_node(node, by, content_only=False):
    parent = node.parentNode

    if by is not None:
        if not isinstance(by, list):
            by = [by]

        # insert new content before node
        for doc in by:
            if content_only:
                c = doc.firstChild
                while c:
                    n = c.nextSibling
                    parent.insertBefore(c, node)
                    c = n
            else:
                parent.insertBefore(doc, node)

    # remove node
    parent.removeChild(node)


def attribute(tag, a):
    """
    Fetch a single attribute value from tag.

    :param tag (xml.dom.Element): DOM element node
    :param a (str): attribute name
    :return: attribute value if present, otherwise None
    """
    if tag.hasAttribute(a):
        # getAttribute returns empty string for non-existent attributes,
        # which makes it impossible to distinguish with empty values
        return tag.getAttribute(a)
    else:
        return None


def opt_attrs(tag, attrs):
    """
    Fetch optional tag attributes.

    Helper routine for fetching optional tag attributes.
    :param tag (xml.dom.Element): DOM element node
    :param attrs [str]: list of attributes to fetch
    """
    return [attribute(tag, a) for a in attrs]


def reqd_attrs(tag, attrs):
    """
    Fetch required tag attributes.

    Helper routine for fetching required tag attributes.
    :param tag (xml.dom.Element): DOM element node
    :param attrs [str]: list of attributes to fetch
    :raise RuntimeError: if required attribute is missing
    """
    result = opt_attrs(tag, attrs)
    for (res, name) in zip(result, attrs):
        if res is None:
            raise RuntimeError(
                '%s: missing attribute "%s"' % (tag.nodeName, name))
    return result


def check_attrs(tag, required, optional):
    """
    Fetch required and optional attributes.

    and complain about any additional attributes.
    :param tag (xml.dom.Element): DOM element node
    :param required [str]: list of required attributes
    :param optional [str]: list of optional attributes
    """
    result = reqd_attrs(tag, required)
    result.extend(opt_attrs(tag, optional))
    allowed = required + optional
    extra = [
        a for a in tag.attributes.keys() if a not in allowed and not a.startswith('xmlns:')]
    if extra:
        warning('%s: unknown attribute(s): %s' %
                (tag.nodeName, ', '.join(extra)))
    return result


# Better pretty printing of xml
# Taken from
# http://ronrothman.com/public/leftbraned/xml-dom-minidom-toprettyxml-and-silly-whitespace/
def fixed_writexml(self, writer, indent='', addindent='', newl=''):
    # indent = current indentation
    # addindent = indentation to add to higher levels
    # newl = newline string
    writer.write(indent + '<' + self.tagName)

    attrs = self._get_attributes()
    a_names = list(attrs.keys())
    a_names.sort()

    for a_name in a_names:
        writer.write(' %s=\"' % a_name)
        xml.dom.minidom._write_data(writer, attrs[a_name].value)
        writer.write('\"')
    if self.childNodes:
        if len(self.childNodes) == 1 \
           and self.childNodes[0].nodeType == xml.dom.minidom.Node.TEXT_NODE:
            writer.write('>')
            self.childNodes[0].writexml(writer, '', '', '')
            writer.write('</%s>%s' % (self.tagName, newl))
            return
        writer.write('>%s' % newl)
        for node in self.childNodes:
            # skip whitespace-only text nodes
            if node.nodeType == xml.dom.minidom.Node.TEXT_NODE and \
                    (not node.data or node.data.isspace()):
                continue
            node.writexml(writer, indent + addindent, addindent, newl)
        writer.write('%s</%s>%s' % (indent, self.tagName, newl))
    else:
        writer.write('/>%s' % newl)


# replace minidom's function with ours
xml.dom.minidom.Element.writexml = fixed_writexml
