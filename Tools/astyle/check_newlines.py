#!/usr/bin/env python3
"""
Check that tracked text files end in exactly one newline: a missing one breaks
the POSIX definition of a line, more than one fails trailing whitespace checks.

Usage:
    check_newlines.py                 check all tracked text files outside docs/
    check_newlines.py FILE...         check only the given files
    check_newlines.py --fix [FILE...] add a missing or remove extra newlines

Only the last bytes of each file are read, so checking the whole tree is fast
enough to run as part of make format.
"""

import argparse
import os
import subprocess
import sys


def text_files():
    """Tracked files git does not consider binary, excluding docs/, relative to the repository root."""
    result = subprocess.run(['git', 'grep', '--cached', '-Il', '-z', '', '--', ':!docs/*'],
                            capture_output=True, check=False)

    if result.returncode not in (0, 1):  # 1: no match
        sys.stderr.write(result.stderr.decode(errors='replace'))
        sys.exit(result.returncode)

    return [p.decode() for p in result.stdout.split(b'\0') if p]


def line_ending(f, size):
    """Trailing '\\r' and '\\n' bytes at the end of the file."""
    ending = b''
    position = size

    while position > 0:
        chunk = min(position, 64)
        position -= chunk
        f.seek(position)
        data = f.read(chunk)
        stripped = data.rstrip(b'\r\n')
        ending = data[len(stripped):] + ending

        if stripped:
            break

    return ending


def check(path, fix):
    try:
        with open(path, 'rb+' if fix else 'rb') as f:
            size = os.fstat(f.fileno()).st_size
            ending = line_ending(f, size)
            newlines = ending.count(b'\n')

            if size == 0 or (newlines == 1 and ending.endswith(b'\n')):
                return True

            if not fix:
                print(f'{"Multiple newlines" if newlines > 1 else "No new line"} at the end of {path}')
                return False

            if newlines == 0:  # replace stray '\r' bytes, if any, by a newline
                f.truncate(size - len(ending))
                f.seek(0, os.SEEK_END)
                f.write(b'\n')

            elif len(ending) == size:  # file contains only line endings
                f.truncate(0)

            else:  # keep the first line ending (including a possible '\r'), drop the rest
                f.truncate(size - len(ending) + ending.index(b'\n') + 1)

            print(f'Fixed newlines at the end of {path}')
            return True

    except (FileNotFoundError, IsADirectoryError):  # deleted or replaced in the working tree
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--fix', action='store_true', help='fix the files in place')
    parser.add_argument('files', nargs='*', help='files to check (default: all tracked text files outside docs/)')
    args = parser.parse_args()

    if args.files:
        files = args.files

    else:
        top = subprocess.run(['git', 'rev-parse', '--show-toplevel'],
                             capture_output=True, text=True, check=True).stdout.strip()
        os.chdir(top)
        files = text_files()

    ok = all([check(path, args.fix) for path in files])
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
