#!/usr/bin/env python3

############################################################################
#
#   Copyright (C) 2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

import argparse
import pathlib
import sys
import os
import tempfile
import json

try:
    from rosidl_generator_type_description import generate_type_hash
except ImportError:
    # modifying sys.path and importing the Python package with the same
    # name as this script does not work on Windows
    rosidl_generator_type_description_root = os.path.dirname(os.path.dirname(__file__))
    rosidl_generator_type_description_module = os.path.join(
        rosidl_generator_type_description_root, 'rosidl_generator_type_description', '__init__.py')
    if not os.path.exists(rosidl_generator_type_description_module):
        raise
    from importlib.machinery import SourceFileLoader

    loader = SourceFileLoader('rosidl_generator_type_description', rosidl_generator_type_description_module)
    rosidl_generator_type_description = loader.load_module()
    generate_type_hash = rosidl_generator_type_description.generate_type_hash

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description=f'Convert px4 .idl files to rihs01')
    parser.add_argument(
        'interface_files', nargs='+',
        help='The interface files to convert')
    parser.add_argument(
        '--output-dir', '-o',
         help='The directory to save converted files (default: current directory)')
    args = parser.parse_args(sys.argv[1:])

    # So for some odd reason rosidl doesn't do proper cli arguments but believes
    # that some magically crafted json is better

    idl_files = []

    type_hash_arguments = {}
    type_hash_arguments['package_name'] = "px4_msgs"
    type_hash_arguments['output_dir'] = args.output_dir
    type_hash_arguments["idl_tuples"] = idl_files

    for interface_file in args.interface_files:
        # So file path need to be magically encoded with a : to let the parser do its thing

        interface_file = str(pathlib.Path(interface_file)).replace("px4/msg", "px4:msg")
        idl_files.append(interface_file)

    json_file = tempfile.NamedTemporaryFile(mode="w+")
    json.dump(type_hash_arguments, json_file)
    json_file.flush()

    generate_type_hash(json_file.name)
