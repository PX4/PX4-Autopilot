#!/usr/bin/env python3
"""
Check that parameter members declared in DEFINE_PARAMETERS() follow the naming
convention _param_<lower case parameter name>, e.g.

    (ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max

Usage:
    check_param_names.py          report violations, exit 1 if there are any
    check_param_names.py --fix    rename violating members

--fix renames the member in the declaring file and its implementation file
(same stem, optionally with _main suffix). If the old name also appears in any
other source file that does not declare it as a parameter member itself, it
might be a use (e.g. a derived class or a helper file) or an unrelated member
with the same name. Telling those apart needs a compiler, so such members are
not renamed but reported for manual renaming, and --fix exits with an error.
This keeps --fix from renaming a declaration while leaving its uses elsewhere
behind, which would break code that compiled before.
"""

import argparse
import os
import re
import subprocess
import sys
from collections import defaultdict

DECLARATION = re.compile(r'<\s*px4::params::(\w+)\s*>\s*\)\s*([A-Za-z_]\w*)')
IDENTIFIER = re.compile(r'[A-Za-z_]\w*')
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


def stem_files(file_stem):
    return [file_stem + suffix + extension
            for suffix in ('', '_main')
            for extension in SOURCE_EXTENSIONS
            if os.path.isfile(file_stem + suffix + extension)]


def all_sources():
    """All tracked C/C++ sources as absolute path -> (identifiers used, parameter members declared)."""
    top = subprocess.run(['git', 'rev-parse', '--show-toplevel'],
                         capture_output=True, text=True, check=True).stdout.strip()
    paths = subprocess.run(['git', 'ls-files', '-z', '--'] + ['*' + e for e in SOURCE_EXTENSIONS],
                           cwd=top, capture_output=True, text=True, check=True).stdout.split('\0')
    sources = {}

    for path in filter(None, paths):
        path = os.path.join(top, path)

        if os.path.isfile(path):
            with open(path, encoding='utf-8', errors='replace', newline='') as f:
                text = f.read()

            sources[path] = (set(IDENTIFIER.findall(text)), {m.group(2) for m in DECLARATION.finditer(text)})

    # a class usually declares its parameters in the header and uses them in the implementation file
    declared_by_stem = defaultdict(set)

    for path, (_, declared) in sources.items():
        declared_by_stem[stem(path)] |= declared

    return {path: (identifiers, declared_by_stem[stem(path)]) for path, (identifiers, _) in sources.items()}


def outside_references(member, own_files, sources):
    """Files outside own_files mentioning member without their stem declaring it as own parameter member."""
    own_files = {os.path.abspath(path) for path in own_files}

    return sorted(os.path.relpath(path) for path, (identifiers, declared) in sources.items()
                  if path not in own_files and member in identifiers and member not in declared)


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

    sources = all_sources() if renames else {}

    for file_stem, stem_renames in renames.items():
        files = stem_files(file_stem)

        for member in list(stem_renames):
            references = outside_references(member, files, sources)

            if references:
                print(f'{member} is also used in {", ".join(references)}, rename to {stem_renames[member]} manually')
                del stem_renames[member]
                ok = False

        if not stem_renames:
            continue

        pattern = re.compile(r'\b(' + '|'.join(map(re.escape, stem_renames)) + r')\b')

        for path in files:
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
