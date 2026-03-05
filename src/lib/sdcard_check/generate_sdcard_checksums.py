#!/usr/bin/env python3

"""Generate SD card file checksums header for firmware integrity verification."""

import argparse
import os


def create_table():
    a = []
    for i in range(256):
        k = i
        for j in range(8):
            if k & 1:
                k ^= 0x1db710640
            k >>= 1
        a.append(k)
    return a


def crc_update(buf, crc_table, crc):
    for k in buf:
        crc = (crc >> 8) ^ crc_table[(crc & 0xff) ^ k]
    return crc


def compute_file_crc(filepath, crc_table):
    crc = 0
    with open(filepath, 'rb') as f:
        while True:
            chunk = f.read(4096)
            if not chunk:
                break
            crc = crc_update(chunk, crc_table, crc)
    return crc


def main():
    parser = argparse.ArgumentParser(
        description='Generate SD card file checksums header')
    parser.add_argument('--sdcard-dir', metavar='DIR',
                        help='SD card build directory to scan')
    parser.add_argument('--output', metavar='FILE', required=True,
                        help='Output header file path')
    args = parser.parse_args()

    entries = []

    if args.sdcard_dir and os.path.isdir(args.sdcard_dir):
        crc_table = create_table()

        for dirpath, dirnames, filenames in os.walk(args.sdcard_dir):
            dirnames.sort()
            for filename in sorted(filenames):
                filepath = os.path.join(dirpath, filename)
                rel_path = os.path.relpath(filepath, args.sdcard_dir)
                crc = compute_file_crc(filepath, crc_table)
                entries.append((rel_path, crc))

    with open(args.output, 'w') as f:
        f.write('#pragma once\n')
        f.write('#include <stdint.h>\n')
        f.write('namespace sdcard_check {\n')
        f.write('struct FileEntry {\n')
        f.write('    const char *relative_path;\n')
        f.write('    uint32_t expected_crc;\n')
        f.write('};\n')
        f.write('static constexpr unsigned num_files = {};\n'.format(
            len(entries)))

        if entries:
            f.write('static constexpr FileEntry files[] = {\n')
            for rel_path, crc in entries:
                f.write('    {{"{}", {}u}},\n'.format(rel_path, crc))
            f.write('};\n')
        else:
            f.write('static constexpr FileEntry *files = nullptr;\n')

        f.write('} // namespace sdcard_check\n')


if __name__ == '__main__':
    main()
