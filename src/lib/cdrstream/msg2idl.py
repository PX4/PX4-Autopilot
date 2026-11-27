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
import builtins
import pathlib
import sys

from rosidl_adapter.msg import convert_msg_to_idl
from rosidl_adapter.parser import parse_message_string
from rosidl_adapter.resource import expand_template


def write_idlc_idl(package_dir, package_name, input_file, output_dir):
    """Write the IDL that CycloneDDS idlc compiles.

    uORB field names such as 'sequence' or 'string' are IDL keywords, which
    idlc rejects. A leading underscore escapes an IDL identifier and idlc strips
    it again, so the generated C names stay unchanged. Escaping every name means
    no keyword list has to track idlc. rosidl_parser rejects escaped names, so
    the RIHS01 hash keeps using the plain IDL from convert_msg_to_idl.
    """
    content = (package_dir / input_file).read_text(encoding='utf-8')
    msg = parse_message_string(package_name, input_file.stem, content)

    for member in msg.fields + msg.constants:
        member.name = f'_{member.name}'

    data = {
        'pkg_name': package_name,
        'relative_input_file': input_file.as_posix(),
        'msg': msg,
    }
    expand_template('msg.idl.em', data, output_dir / input_file.with_suffix('.idl').name,
                    encoding='iso-8859-1')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description=f'Convert px4 .msg files to .idl')
    parser.add_argument(
        'interface_files', nargs='+',
        help='The interface files to convert')
    parser.add_argument(
        '--idlc-output-dir', type=pathlib.Path, required=True,
        help='Directory for the keyword-escaped IDL compiled by idlc')
    args = parser.parse_args(sys.argv[1:])

    # convert_msg_to_idl prints a Reading/Writing line per file. Filter those
    # via builtins.print instead of redirecting stdout, which the empy template
    # engine relies on and breaks if replaced. Errors still surface on stderr.
    real_print = builtins.print

    def quiet_print(*pargs, **kwargs):
        if pargs and isinstance(pargs[0], str) and \
                pargs[0].startswith(('Reading input file:', 'Writing output file:')):
            return
        real_print(*pargs, **kwargs)

    builtins.print = quiet_print
    try:
        for interface_file in args.interface_files:
            interface_file = pathlib.Path(interface_file)
            package_dir = interface_file.parent.absolute()
            input_file = interface_file.absolute().relative_to(package_dir)

            convert_msg_to_idl(
                package_dir, 'px4_msgs', input_file, interface_file.parent)
            write_idlc_idl(package_dir, 'px4_msgs', input_file, args.idlc_output_dir)
    finally:
        builtins.print = real_print
