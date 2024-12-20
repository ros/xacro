# Copyright (c) 2024, Open Source Robotics Foundation, Inc.
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
#     * Neither the name of the Open Source Robotics Foundation, Inc.
#       nor the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior
#       written permission.
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

"""Support methods for xacro.

Since xacro needs to access the file system for its include functionality, we
need to provide an implementation for dealing with files in bazel.
"""

import os
import errno

def open_bazel(path, *args, **kwargs):
    """Open a file.

    If path starts with '//', assume it's a bazel path and look for it in
    the list of input files (in os.environ['XACRO_INPUTS']).

    Args:
    path: Path of the file to open. If this starts with '//', assume it's a
          bazel path
    *args: passed through to open()
    **kwargs: passed through to open()

    Raises:
    IOError: If the requested path is not available either on the filesystem or
             in bazel.
    """
    if path.startswith('//'):
        # Get path of file, without leading //
        bazel_path = path[2:]
        # Extract list of input files from environment variable
        input_paths = os.environ['XACRO_INPUTS'].split('\n')
        best_match = None
        # Find the match that is shortest overall – this avoids selecting
        # not/my/package/file.xacro when we really want my/package/file.xacro.
        for input_path in input_paths:
            if input_path.endswith(bazel_path):
                if best_match is None or len(best_match) > len(input_path):
                    best_match = input_path

        if best_match is None:
            raise IOError(
              errno.ENOENT,
              'Unable to find bazel file. Is it or the rule that produces it an input?',
              path)
        path = best_match

    return open(path, *args, **kwargs)
