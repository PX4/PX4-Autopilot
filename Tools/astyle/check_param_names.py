#!/usr/bin/env python3
"""
Check that parameter members declared in DEFINE_PARAMETERS() follow the naming
convention _param_<lower case parameter name>, e.g.

    (ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max

Usage:
    check_param_names.py          report violations, exit 1 if there are any
    check_param_names.py --fix    rename violating members

--fix renames the member in the declaring file and its implementation file
(same stem, optionally with _main suffix). The same wrong name is often used by
other classes for unrelated members, so uses elsewhere (e.g. derived classes)
are left for the compiler to find.
"""

import argparse
import os
import re
import subprocess
import sys
from collections import defaultdict

DECLARATION = re.compile(r'<\s*px4::params::(\w+)\s*>\s*\)\s*([A-Za-z_]\w*)')
SOURCE_EXTENSIONS = ('.c', '.cpp', '.h', '.hpp')


def expected_name(param):
    return '_param_' + param.lower()


def find_violations(files):
    violations = []

    for path in files:
        with open(path, encoding='utf-8', errors='replace', newline='') as f:
            text = f.read()

        if 'px4::params::' not in text:
            continue

        for match in DECLARATION.finditer(text):
            param, member = match.groups()

            if member != expected_name(param):
                line = text.count('\n', 0, match.start(2)) + 1
                violations.append((path, line, param, member))

    return violations


def stem(path):
    name = os.path.splitext(path)[0]
    return name[:-len('_main')] if name.endswith('_main') else name


def fix(violations):
    # old name -> new name per file stem
    renames = defaultdict(dict)
    ok = True

    for path, line, param, member in violations:
        stem_renames = renames[stem(path)]
        new = expected_name(param)

        if stem_renames.get(member, new) != new:
            print(f'{path}:{line}: {member} maps to both {stem_renames[member]} and {new}, rename manually')
            ok = False
            continue

        stem_renames[member] = new

    for file_stem, stem_renames in renames.items():
        pattern = re.compile(r'\b(' + '|'.join(map(re.escape, stem_renames)) + r')\b')

        for suffix in ('', '_main'):
            for extension in SOURCE_EXTENSIONS:
                path = file_stem + suffix + extension

                if not os.path.isfile(path):
                    continue

                with open(path, encoding='utf-8', newline='') as f:
                    text = f.read()

                new_text = pattern.sub(lambda m: stem_renames[m.group(1)], text)

                if new_text != text:
                    with open(path, 'w', encoding='utf-8', newline='') as f:
                        f.write(new_text)

                    print(f'renamed parameter members in {path}')

    return ok


def main():
    parser = argparse.ArgumentParser(description='Check parameter member naming')
    parser.add_argument('--fix', action='store_true', help='rename violating members')
    parser.add_argument('files', nargs='*', help='files to check (default: all checked by astyle)')
    args = parser.parse_args()

    files = args.files

    if not files:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        os.chdir(os.path.join(script_dir, '..', '..'))
        files = subprocess.run([os.path.join(script_dir, 'files_to_check_code_style.sh')],
                               capture_output=True, text=True, check=True).stdout.split()

    violations = find_violations(files)

    if args.fix:
        return 0 if fix(violations) else 1

    for path, line, param, member in violations:
        print(f'{path}:{line}: parameter member {member} should be named {expected_name(param)}')

    if violations:
        print(f'{len(violations)} parameter members not named _param_<lower case parameter name>, '
              'run `make format` to fix')
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
