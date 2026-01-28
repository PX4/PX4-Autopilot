#!/usr/bin/env python3
#############################################################################
#
#   Copyright (C) 2023 PX4 Pro Development Team. All rights reserved.
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
#############################################################################

"""
Generates cpp source + header files with compressed uorb topic fields from json files
"""

import argparse
import json
import struct
from operator import itemgetter
import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../src/lib/heatshrink'))
import heatshrink_encode


def parse_json_files(json_files: [str]) -> dict:
    """Read list of json files into a dict"""
    definitions = {}
    for json_file in json_files:
        with open(json_file, encoding='utf-8') as file_handle:
            definition = json.load(file_handle)
            assert definition['name'] not in definitions
            definitions[definition['name']] = definition
            definitions[definition['name']]['completed'] = False

    return definitions


def get_ordered_list_by_dependency(name: str, definitions: dict) -> [str]:
    """Iterate dependency graph and create an ordered list"""
    if definitions[name]['completed']:
        return []
    ret = []
    # Get nested types first (DFS)
    for dependency in definitions[name]['dependencies']:
        ret.extend(get_ordered_list_by_dependency(dependency, definitions))

    ret.append(name)
    definitions[name]['completed'] = True
    return ret


def get_field_definitions(names: [str], definitions: dict) -> (bytes, [str]):
    """Get byte array with all definitions"""
    ret = bytes()
    formats_list = []

    for name in names:
        # Format as '<# orb_ids><orb_id0...><# orb_ids dependencies<orb_id_dependency0...><fields><null>'
        assert len(definitions[name]['orb_ids']) < 255
        assert len(definitions[name]['dependencies']) < 255
        ret += struct.pack('<B', len(definitions[name]['orb_ids']))
        for orb_id in definitions[name]['orb_ids']:
            assert orb_id < (1 << 16)
            ret += struct.pack('<H', orb_id)
        # Dependencies
        ret += struct.pack('<B', len(definitions[name]['dependencies']))
        for dependent_message_name in definitions[name]['dependencies']:
            # Get ORB ID by looking up the name in all definitions
            dependent_orb_id_list = [definitions[k]['main_orb_id'] for k in definitions if
                                     definitions[k]['name'] == dependent_message_name]
            assert len(dependent_orb_id_list) == 1
            orb_id = dependent_orb_id_list[0]
            assert (1 << 16) > orb_id >= 0
            ret += struct.pack('<H', orb_id)

        ret += bytes(definitions[name]['fields'], 'latin1')
        ret += b'\0'

        formats_list.append(definitions[name]['fields'])

    return ret, formats_list


def write_fields_to_cpp_file(file_name: str, compressed_fields: bytes):
    fields_str = ', '.join(str(c) for c in compressed_fields)
    with open(file_name, 'w') as file_handle:
        file_handle.write('''
// Auto-generated from px4_generate_uorb_compressed_fields.py
#include <uORB/topics/uORBMessageFieldsGenerated.hpp>

namespace uORB {

static const uint8_t compressed_fields[] = {
    {FIELDS}
};

const uint8_t* orb_compressed_message_formats()
{
    return compressed_fields;
}
unsigned orb_compressed_message_formats_size()
{
    return sizeof(compressed_fields) / sizeof(compressed_fields[0]);
}

} // namespace uORB
'''.replace('{FIELDS}', fields_str))


def c_encode(s, encoding='ascii'):
    result = ''
    for c in s:
        if not (32 <= ord(c) < 127) or c in ('\\', '"'):
            result += '\\%03o' % ord(c)
        else:
            result += c
    return '"' + result + '"'


def write_fields_to_hpp_file(file_name: str, definitions: dict, window_length: int, lookahead_length: int,
                             format_list: [str]):
    max_tokenized_field_length, max_tokenized_field_length_msg = max(
        ((len(definitions[k]['fields']), k) for k in definitions), key=itemgetter(0))
    max_untokenized_field_length = max(definitions[k]['fields_total_length'] for k in definitions)
    max_num_orb_ids = max(len(definitions[k]['orb_ids']) for k in definitions)
    max_num_orb_id_dependencies = max(len(definitions[k]['dependencies']) for k in definitions)

    with open(file_name, 'w') as file_handle:
        file_handle.write('''
// Auto-generated from px4_generate_uorb_compressed_fields.py
#include <cstdint>

namespace uORB {

 /**
  * Get compressed string of all uorb message format definitions
  */
const uint8_t* orb_compressed_message_formats();

/**
 * Get length of compressed message format definitions
 */
unsigned orb_compressed_message_formats_size();

static constexpr unsigned orb_tokenized_fields_max_length = {MAX_TOKENIZED_FIELD_LENGTH}; // {MAX_TOKENIZED_FIELD_LENGTH_MSG}
static constexpr unsigned orb_untokenized_fields_max_length = {MAX_UNTOKENIZED_FIELD_LENGTH};
static constexpr unsigned orb_compressed_max_num_orb_ids = {MAX_NUM_ORB_IDS};
static constexpr unsigned orb_compressed_max_num_orb_id_dependencies = {MAX_NUM_ORB_ID_DEPENDENCIES};

static constexpr unsigned orb_compressed_heatshrink_window_length = {WINDOW_LENGTH};
static constexpr unsigned orb_compressed_heatshrink_lookahead_length = {LOOKAHEAD_LENGTH};

#define ORB_DECOMPRESSED_MESSAGE_FIELDS {{DECOMPRESSED_MESSAGE_FIELDS}}

} // namespace uORB
'''
                          .replace('{MAX_TOKENIZED_FIELD_LENGTH}', str(max_tokenized_field_length))
                          .replace('{MAX_TOKENIZED_FIELD_LENGTH_MSG}', max_tokenized_field_length_msg)
                          .replace('{MAX_UNTOKENIZED_FIELD_LENGTH}', str(max_untokenized_field_length))
                          .replace('{MAX_NUM_ORB_IDS}', str(max_num_orb_ids))
                          .replace('{MAX_NUM_ORB_ID_DEPENDENCIES}', str(max_num_orb_id_dependencies))
                          .replace('{WINDOW_LENGTH}', str(window_length))
                          .replace('{LOOKAHEAD_LENGTH}', str(lookahead_length))
                          .replace('{DECOMPRESSED_MESSAGE_FIELDS}', ','.join(c_encode(x) for x in format_list))
                          )


def main():
    parser = argparse.ArgumentParser(description='Generate compressed uorb topic fields')
    parser.add_argument('-f', dest='file',
                        help="json input files",
                        nargs="+")
    parser.add_argument('--source-output-file', dest='output_cpp',
                        help='cpp output file to generate')
    parser.add_argument('--header-output-file', dest='output_hpp',
                        help='hpp output file to generate')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        help="verbose output")
    args = parser.parse_args()

    if args.file is not None:
        definitions = parse_json_files(args.file)

        # Get array of all field definitions
        names = []
        for definition in definitions:
            names.extend(get_ordered_list_by_dependency(definitions[definition]['name'], definitions))
        names.reverse()  # Dependent definitions must be after
        assert len(names) == len(definitions)
        for definition in definitions:  # sanity check
            assert definitions[definition]['completed']
        field_definitions, format_list = get_field_definitions(names, definitions)

        # Compress
        window_size = 8  # Larger value = better compression; memory requirement (for decompression): 2 ^ window_size
        lookahead = 4
        compressed_field_definitions = heatshrink_encode.encode(field_definitions, window_size, lookahead)

        if args.verbose:
            print(
                f'Field definitions: size: {len(field_definitions)}, reduction from compression: {len(field_definitions) - len(compressed_field_definitions)}')

        # Write cpp & hpp file
        write_fields_to_cpp_file(args.output_cpp, compressed_field_definitions)
        write_fields_to_hpp_file(args.output_hpp, definitions, window_size, lookahead, format_list)


if __name__ == "__main__":
    main()
