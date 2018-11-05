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
#
# Revision $Id: substitution_args.py 15178 2011-10-10 21:22:53Z kwc $

"""
Library for processing XML substitution args.

This is currently used by roslaunch and xacro,
but it is not yet a top-level ROS feature.
This script has been taken from catkin's
ros_comm/tools/roslaunch/src/roslaunch/substitution_args.py
"""

import math
import os
import sys

import yaml

try:
    from cStringIO import StringIO  # Python 2.x
except ImportError:
    from io import StringIO  # Python 3.x

# import rosgraph.names  # TODO Use this when rosgraph will be migrated to ROS2.
# import c
# from roslaunch.loader import convert_value

_rospack = None


class SubstitutionException(Exception):
    """Base class for exceptions in substitution_args routines."""

    pass


class ArgException(SubstitutionException):
    """Exception for missing $(arg) values."""

    pass


def _eval_env(name):
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


def anonymous_name(id):
    """
    Generate a ROS-legal 'anonymous' name.

    @param id: prefix for anonymous name
    @type  id: str
    """
    import socket
    import random
    name = '%s_%s_%s_%s' % (
        id, socket.gethostname(), os.getpid(), random.randint(0, sys.maxsize))
    # RFC 952 allows hyphens, IP addrs can have '.'s, both
    # of which are illegal for ROS names. For good
    # measure, screen ipv6 ':'.
    name = name.replace('.', '_')
    name = name.replace('-', '_')
    return name.replace(':', '_')


def _eval_anon(id, anons):

    if id in anons:
        return anons[id]
    # TODO use this when rograph will be available in ROS2.
    # resolve_to = rosgraph.names.anonymous_name(id)
    # temporary fix, remove this when rosgraph will be migrated to ROS2.
    resolve_to = anonymous_name(id)
    anons[id] = resolve_to
    return resolve_to


def _anon(resolved, a, args, context):
    """
    Process $(anon) arg.

    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    # 1559 #1660
    if len(args) == 0:
        raise SubstitutionException('$(anon var) must specify a name [%s]' % a)
    elif len(args) > 1:
        raise SubstitutionException(
            '$(anon var) may only specify one name [%s]' % a)
    if 'anon' not in context:
        context['anon'] = {}
    anon_context = context['anon']
    return resolved.replace('$(%s)' % a, _eval_anon(id=args[0], anons=anon_context))


def _eval_dirname(filename):

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
    rp = _get_rospack()
    return rp.get_path(pkg)


def _find(resolved, a, args, context):
    """
    Process $(find PKG).

    Resolves the path while considering the path following the command
    to provide backward compatible results.
    If it is followed by a path it first tries to resolve it
    as an executable and than as a normal file under share.
    Else it resolves to the source share folder of the PKG.
    :returns: updated resolved argument, ``str``
    :raises: :exc:SubstitutionException: if PKG invalidly specified
    :raises: :exc:`rospkg.ResourceNotFound` If PKG requires resource (e.g. package)
    that does not exist
    """
    if len(args) != 1:
        raise SubstitutionException(
            '$(find pkg) command only accepts one argument [%s]' % a)
    before, after = _split_command(resolved, a)
    path, after = _separate_first_path(after)
    resolve_without_path = before + ('$(%s)' % a) + after
    path = _sanitize_path(path)
    if path.startswith('/') or path.startswith('\\'):
        path = path[1:]
    rp = _get_rospack()
    if path:
        source_path_to_packages = rp.get_custom_cache(
            'source_path_to_packages', {})
        res = None
        try:
            res = _find_executable(
                resolve_without_path, a, [args[0], path], context,
                source_path_to_packages=source_path_to_packages)
        except SubstitutionException:
            pass
        if res is None:
            try:
                res = _find_resource(
                    resolve_without_path, a, [args[0], path], context,
                    source_path_to_packages=source_path_to_packages)
            except SubstitutionException:
                pass
        # persist mapping of packages in rospack instance
        if source_path_to_packages:
            rp.set_custom_cache(
                'source_path_to_packages', source_path_to_packages)
        if res is not None:
            return res
    pkg_path = rp.get_path(args[0])
    if path:
        pkg_path = os.path.join(pkg_path, path)
    return before + pkg_path + after


def _find_executable(resolved, a, args, _context, source_path_to_packages=None):
    """
    Process $(find-executable PKG PATH).

    It finds the executable with the basename(PATH) in the libexec folder
    or under the PATH relative to the package.xml file.
    :returns: updated resolved argument, ``str``
    :raises: :exc:SubstitutionException: if PKG/PATH invalidly specified
    or executable is not found for PKG
    """
    if len(args) != 2:
        raise SubstitutionException('$(find-executable pkg path) '
                                    'command only accepts two argument [%s]' % a)
    before, after = _split_command(resolved, a)
    path = _sanitize_path(args[1])
    # we try to find the specific executable in libexec via catkin
    # which will search in install/devel space
    full_path = None
    from catkin.find_in_workspaces import find_in_workspaces
    paths = find_in_workspaces(
        ['libexec'], project=args[0], first_matching_workspace_only=True,
        # implicitly first_match_only=True
        source_path_to_packages=source_path_to_packages)
    if paths:
        full_path = _get_executable_path(paths[0], os.path.basename(path))
    if not full_path:
        # else we will look for the executable in the source folder of the
        # package
        rp = _get_rospack()
        full_path = _get_executable_path(rp.get_path(args[0]), path)
    if not full_path:
        raise SubstitutionException('$(find-executable pkg path) '
                                    'could not find executable [%s]' % a)
    return before + full_path + after


def _find_resource(resolved, a, args, _context, source_path_to_packages=None):
    """
    Process $(find-resource PKG PATH).

    Resolves the relative PATH from the share folder of the PKG either from install space,
    devel space or from the source folder.
    :returns: updated resolved argument, ``str``
    :raises: :exc:SubstitutionException: if PKG and PATH invalidly specified
    or relative PATH is not found for PKG
    """
    if len(args) != 2:
        raise SubstitutionException('$(find-resource pkg path) '
                                    'command only accepts two argument [%s]' % a)
    before, after = _split_command(resolved, a)
    path = _sanitize_path(args[1])
    # we try to find the specific path in share via catkin
    # which will search in install/devel space and the source folder of the
    # package
    from catkin.find_in_workspaces import find_in_workspaces
    paths = find_in_workspaces(
        ['share'], project=args[0], path=path, first_matching_workspace_only=True,
        first_match_only=True, source_path_to_packages=source_path_to_packages)
    if not paths:
        raise SubstitutionException(
            '$(find-resource pkg path) could not find path [%s]' % a)
    return before + paths[0] + after


def _split_command(resolved, command_with_args):

    cmd = '$(%s)' % command_with_args
    idx1 = resolved.find(cmd)
    idx2 = idx1 + len(cmd)
    return resolved[0:idx1], resolved[idx2:]


def _separate_first_path(value):

    idx = value.find(' ')
    if idx < 0:
        path, rest = value, ''
    else:
        path, rest = value[0:idx], value[idx:]
    return path, rest


def _sanitize_path(path):

    path = path.replace('/', os.sep)
    path = path.replace('\\', os.sep)
    return path


def _get_executable_path(base_path, path):

    full_path = os.path.join(base_path, path)
    if os.path.isfile(full_path) and os.access(full_path, os.X_OK):
        return full_path
    return None


def _get_rospack():

    global _rospack
    '''
    if _rospack is None:
        _rospack = rospkg.RosPack()
    '''
    return _rospack


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

    if 'anon' not in context:
        context['anon'] = {}
    if 'arg' not in context:
        context['arg'] = {}

    # inject correct anon context
    def _eval_anon_context(id):
        return _eval_anon(id, anons=context['anon'])

    # inject arg context
    def _eval_arg_context(name):
        return convert_value(_eval_arg(name, args=context['arg']), 'auto')

    # inject dirname context
    def _eval_dirname_context():
        return _eval_dirname(context['filename'])

    functions = {
        'anon': _eval_anon_context,
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


def resolve_args(arg_str, context=None, resolve_anon=True, filename=None):
    """
    Resolve substitution args (see wiki spec U{http://ros.org/wiki/roslaunch}).

    @param arg_str: string to resolve zero or more substitution args
        in. arg_str may be None, in which case resolve_args will
        return None
    @type  arg_str: str
    @param context dict: (optional) dictionary for storing results of
        the 'anon' and 'arg' substitution args. multiple calls to
        resolve_args should use the same context so that 'anon'
        substitions resolve consistently. If no context is provided, a
        new one will be created for each call. Values for the 'arg'
        context should be stored as a dictionary in the 'arg' key.
    @type  context: dict
    @param resolve_anon bool: If True (default), will resolve $(anon
        foo). If false, will leave these args as-is.
    @type  resolve_anon: bool

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
        'anon': _anon,
        'arg': _arg,
    }
    resolved = _resolve_args(arg_str, context, resolve_anon, commands)
    # then resolve 'find' as it requires the subsequent path to be expanded
    # already
    commands = {
        'find': _find,
    }
    resolved = _resolve_args(resolved, context, resolve_anon, commands)
    return resolved


def _resolve_args(arg_str, context, resolve_anon, commands):

    valid = ['find', 'env', 'optenv', 'dirname', 'anon', 'arg']
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
