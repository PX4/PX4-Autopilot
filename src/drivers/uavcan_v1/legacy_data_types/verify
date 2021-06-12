#!/usr/bin/env python3

import sys
import string
import pydsdl
from functools import partial


MAX_LINE_LENGTH = 120
ALLOWED_CHARACTERS = set(string.digits + string.ascii_letters + string.punctuation + ' ')


def die_at(ty, line_index, *text):
    prefix = '%s:%d:' % (ty.source_file_path, line_index + 1)
    print(prefix, *text, file=sys.stderr)
    sys.exit(1)


def on_print(file_path, line, value):
    print('%s:%d: %s' % (file_path, line, value), file=sys.stderr)


ns_list = [
    'uavcan',
    'reg',
]
output = []
for ns in ns_list:
    output += pydsdl.read_namespace(ns, ns_list, print_output_handler=on_print)

longest_name = max(map(lambda x: len(str(x)), output))
print('Full data type name'.ljust(longest_name), 'Fixed port-ID', 'Bytes'.ljust(10), sep='\t')
for t in output:
    if isinstance(t, pydsdl.ServiceType):
        n_bytes = ' '.join([f'{max(x.bit_length_set) // 8: 4}' for x in (t.request_type, t.response_type)])
    else:
        n_bytes = str(max(t.bit_length_set) // 8)

    print(str(t).ljust(longest_name),
          str(t.fixed_port_id if t.has_fixed_port_id else '').rjust(13),
          n_bytes.rjust(10) + ' ',
          sep='\t')

for t in output:
    text = open(t.source_file_path).read()
    for index, line in enumerate(text.split('\n')):
        line = line.strip('\r\n')
        abort = partial(die_at, t, index)

        # Check trailing comment placement.
        # TODO: this test breaks on string literals containing "#".
        if not line.startswith('#') and '#' in line and '  #' not in line:
            abort('Trailing line comments shall be separated from the preceding text with at least two spaces')

        if line != '#' and '#' in line and '# ' not in line:
            abort('The text of a comment shall be separated from the comment character with a single space')

        if line.endswith(' '):
            abort('Trailing spaces are not permitted')

        # Check line length limit
        if len(line) > MAX_LINE_LENGTH:
            abort('Line is too long:', len(line), '>', MAX_LINE_LENGTH, 'chars')

        # Make sure we're not using any weird characters such as tabs or non-ASCII-printable
        for char_index, char in enumerate(line):
            if char not in ALLOWED_CHARACTERS:
                abort('Disallowed character', repr(char), 'code', ord(char), 'at column', char_index + 1)

    if not text.endswith('\n') or text.endswith('\n' * 2):
        abort('A file shall contain exactly one blank line at the end')
