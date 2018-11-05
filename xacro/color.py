#!/usr/bin/env python3

# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from __future__ import print_function

import sys

# bold colors
_ansi = {'red': 91, 'yellow': 93}


def is_tty(stream):
    """Return True if the given stream is a tty, else False."""
    return hasattr(stream, 'isatty') and stream.isatty()


def colorize(msg, color, file=sys.stderr, alt_text=None):
    if color and is_tty(file):
        return '\033[%dm%s\033[0m' % (_ansi[color], msg)
    elif alt_text:
        return '%s%s' % (alt_text, msg)
    else:
        return msg


def message(msg, *args, **kwargs):
    file = kwargs.get('file', sys.stderr)
    alt_text = kwargs.get('alt_text', None)
    color = kwargs.get('color', None)
    print(colorize(msg, color, file, alt_text), *args, file=file)


def warning(*args, **kwargs):
    # defaults = dict(file=sys.stderr, alt_text='warning: ', color='yellow')
    # Unnecessary dict call - rewrite as a literal.
    defaults = {'file': sys.stderr, 'alt_text': 'warning: ', 'color': 'yellow'}
    defaults.update(kwargs)
    message(*args, **defaults)


def error(*args, **kwargs):
    # defaults = dict(file=sys.stderr, alt_text='error: ', color='red')
    defaults = {'file': sys.stderr, 'alt_text': 'error: ', 'color': 'red'}
    defaults.update(kwargs)
    message(*args, **defaults)
