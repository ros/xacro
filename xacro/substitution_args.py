# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Library for processing XML substitution args.

This is currently used by roslaunch and xacro, but it is not yet a top-level ROS feature.
This file has been modified from ros_comm/tools/roslaunch/src/roslaunch/substitution_args.py
"""

import math
import os
import yaml

from io import StringIO


class SubstitutionException(Exception):
    """Base class for exceptions in substitution_args routines."""
    pass


class ArgException(SubstitutionException):
    """Exception for missing $(arg) values."""
    pass


def _eval_env(name):
    """
    Returns the environment variable value or throws exception.

    @return: enviroment variable value
    @raise SubstitutionException: if environment variable not set
    """
    try:
        return os.environ[name]
    except KeyError as e:
        raise SubstitutionException(
            'environment variable %s is not set' % str(e))


def _env(resolved, a, args, context):
    """
    Process $(env) arg.

    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) != 1:
        raise SubstitutionException(
            '$(env var) command only accepts one argument [%s]' % a)
    return resolved.replace('$(%s)' % a, _eval_env(args[0]))


def _eval_optenv(name, default=''):
    """
    Eval_optenv

    Returns the value of the environment variable or default

    @name: name of the environment variable
    @return: enviroment variable value or default
    """
    if name in os.environ:
        return os.environ[name]
    return default


def _optenv(resolved, a, args, context):
    """
    Process $(optenv) arg.

    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException(
            '$(optenv var) must specify an environment variable [%s]' % a)
    return resolved.replace('$(%s)' % a, _eval_optenv(args[0], default=' '.join(args[1:])))


def _eval_dirname(filename):
    """
    Gets the absolute path of a given filename

    @param filename
    @return: absolute path
    @rtype path
    """
    if not filename:
        raise SubstitutionException('Cannot substitute $(dirname),'
                                    'no file/directory information available.')
    return os.path.abspath(os.path.dirname(filename))


def _dirname(resolved, a, args, context):
    """
    Process $(dirname).

    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if no information about the current launch file is available,
    for example if XML was passed via stdin, or this is a remote launch.
    """
    return resolved.replace('$(%s)' % a, _eval_dirname(context.get('filename', None)))


def _eval_find(pkg):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


def _find(resolved, a, args, context):
    """
    Process $(find PKG).

    Resolves to the share folder of the package
    :returns: updated resolved argument, ``str``
    :raises: :exc:SubstitutionException: if PKG invalidly specified
    """
    if len(args) != 1:
        raise SubstitutionException(
            '$(find pkg) accepts exactly one argument [%s]' % a)
    return resolved.replace('$(%s)' % a, _eval_find(args[0]))


def _eval_arg(name, args):

    try:
        return args[name]
    except KeyError:
        raise ArgException(name)


def _arg(resolved, a, args, context):
    """
    Process $(arg) arg.

    :returns: updated resolved argument, ``str``
    :raises: :exc:`ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException(
            '$(arg var) must specify a variable name [%s]' % (a))
    elif len(args) > 1:
        raise SubstitutionException(
            '$(arg var) may only specify one arg [%s]' % (a))

    if 'arg' not in context:
        context['arg'] = {}
    return resolved.replace('$(%s)' % a, _eval_arg(name=args[0], args=context['arg']))


# Create a dictionary of global symbols that will be available in the eval
# context.  We disable all the builtins, then add back True and False, and also
# add true and false for convenience (because we accept those lower-case strings
# as boolean values in XML).
_eval_dict = {
    'true': True, 'false': False,
    'True': True, 'False': False,
    '__builtins__': {k: __builtins__[k] for k in ['list', 'dict', 'map', 'str', 'float', 'int']},
    'env': _eval_env,
    'optenv': _eval_optenv,
    'find': _eval_find
}
# also define all math symbols and functions
_eval_dict.update(math.__dict__)


def convert_value(value, type_):
    """
    Convert a value from a string representation into the specified type.

    @param value: string representation of value
    @type  value: str
    @param type_: int, double, string, bool, or auto
    @type  type_: str
    @raise ValueError: if parameters are invalid
    """
    type_ = type_.lower()
    # currently don't support XML-RPC date, dateTime, maps, or list
    # types
    if type_ == 'auto':
        # attempt numeric conversion
        try:
            if '.' in value:
                return float(value)
            else:
                return int(value)
        except ValueError:
            pass
        # bool
        lval = value.lower()
        if lval == 'true' or lval == 'false':
            return convert_value(value, 'bool')
        # string
        return value
    elif type_ == 'str' or type_ == 'string':
        return value
    elif type_ == 'int':
        return int(value)
    elif type_ == 'double':
        return float(value)
    elif type_ == 'bool' or type_ == 'boolean':
        value = value.lower().strip()
        if value == 'true' or value == '1':
            return True
        elif value == 'false' or value == '0':
            return False
        raise ValueError("%s is not a '%s' type" % (value, type_))
    elif type_ == 'yaml':
        try:
            return yaml.load(value)
        except yaml.parser.ParserError as e:
            raise ValueError(e)
    else:
        raise ValueError("Unknown type '%s'" % type_)


class _DictWrapper(object):

    def __init__(self, args, functions):
        self._args = args
        self._functions = functions

    def __getitem__(self, key):
        try:
            return self._functions[key]
        except KeyError:
            return convert_value(self._args[key], 'auto')


def _eval(s, context):

    if 'arg' not in context:
        context['arg'] = {}

    # inject arg context
    def _eval_arg_context(name):
        return convert_value(_eval_arg(name, args=context['arg']), 'auto')

    # inject dirname context
    def _eval_dirname_context():
        return _eval_dirname(context['filename'])

    functions = {
        'arg': _eval_arg_context,
        'dirname': _eval_dirname_context
    }
    functions.update(_eval_dict)

    # ignore values containing double underscores (for safety)
    # http://nedbatchelder.com/blog/201206/eval_really_is_dangerous.html
    if s.find('__') >= 0:
        raise SubstitutionException(
            '$(eval ...) may not contain double underscore expressions')
    return str(eval(s, {}, _DictWrapper(context['arg'], functions)))


def resolve_args(arg_str, context=None, filename=None):
    """
    Resolve substitution args (see wiki spec U{http://ros.org/wiki/roslaunch}).

    @param arg_str: string to resolve zero or more substitution args in.
        arg_str may be None, in which case resolve_args will return None
    @type  arg_str: str
    @param context dict: (optional) dictionary for storing results of the 'arg' substitution args.
        If no context is provided, a new one will be created for each call. Values for the 'arg'
        context should be stored as a dictionary in the 'arg' key.
    @type  context: dict

    @return str: arg_str with substitution args resolved
    @rtype:  str
    @raise SubstitutionException: if there is an error resolving substitution args
    """
    if context is None:
        context = {}
    if not arg_str:
        return arg_str
    # special handling of $(eval ...)
    if arg_str.startswith('$(eval ') and arg_str.endswith(')'):
        return _eval(arg_str[7:-1], context)
    # first resolve variables like 'env' and 'arg'
    commands = {
        'env': _env,
        'optenv': _optenv,
        'dirname': _dirname,
        'arg': _arg,
        'find': _find,
    }
    resolved = _resolve_args(arg_str, context, commands)
    # then resolve 'find' as it requires the subsequent path to be expanded already
    commands = {
        "find": _find,
    }
    resolved = _resolve_args(resolved, context, commands)
    return resolved


def _resolve_args(arg_str, context, commands):

    valid = ['find', 'env', 'optenv', 'dirname', 'arg']
    resolved = arg_str
    for a in _collect_args(arg_str):
        splits = [s for s in a.split(' ') if s]
        if not splits[0] in valid:
            raise SubstitutionException('Unknown substitution command [%s]. '
                                        'Valid commands are %s' % (a, valid))
        command = splits[0]
        args = splits[1:]
        if command in commands:
            resolved = commands[command](resolved, a, args, context)
    return resolved


_OUT = 0
_DOLLAR = 1
_LP = 2
_IN = 3


def _collect_args(arg_str):
    """
    State-machine parser for resolve_args.

    Substitution args are of the form:
    $(find package_name)/scripts/foo.py $(export some/attribute blar) non-relevant stuff

    @param arg_str: argument string to parse args from
    @type  arg_str: str
    @raise SubstitutionException: if args are invalidly specified
    @return: list of arguments
    @rtype: [str]
    """
    buff = StringIO()
    args = []
    state = _OUT
    for c in arg_str:
        # No escapes supported
        if c == '$':
            if state == _OUT:
                state = _DOLLAR
            elif state == _DOLLAR:
                pass
            else:
                raise SubstitutionException('Dollar signs "$" cannot be '
                                            'inside of substitution args [%s]' % arg_str)
        elif c == '(':
            if state == _DOLLAR:
                state = _LP
            elif state != _OUT:
                raise SubstitutionException('Invalid left parenthesis "(" '
                                            'in substitution args [%s]' % arg_str)
        elif c == ')':
            if state == _IN:
                # save contents of collected buffer
                args.append(buff.getvalue())
                buff.truncate(0)
                buff.seek(0)
                state = _OUT
            else:
                state = _OUT
        elif state == _DOLLAR:
            # left paren must immediately follow dollar sign to enter _IN state
            state = _OUT
        elif state == _LP:
            state = _IN

        if state == _IN:
            buff.write(c)
    return args
