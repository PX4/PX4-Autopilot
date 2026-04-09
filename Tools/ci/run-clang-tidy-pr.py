#!/usr/bin/env python3
"""
Run clang-tidy incrementally on files changed in a PR.

Usage: run-clang-tidy-pr.py <base-ref>
  base-ref: e.g. origin/main

Computes the set of translation units (TUs) affected by the PR diff,
then invokes Tools/run-clang-tidy.py on that subset only.
Exits 0 silently when no C++ files were changed.
"""

import argparse
import json
import os
import subprocess
import sys

EXTENSIONS_CPP = {'.cpp', '.c'}
EXTENSIONS_HDR = {'.hpp', '.h'}
# Manual exclusions from Makefile:508
EXCLUDE_EXTRA = '|'.join([
    'src/systemcmds/tests',
    'src/examples',
    'src/modules/gyro_fft/CMSIS_5',
    'src/lib/drivers/smbus',
    'src/drivers/gpio',
    r'src/modules/commander/failsafe/emscripten',
    r'failsafe_test\.dir',
])


def repo_root():
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', '--show-toplevel'], text=True).strip()
    except subprocess.CalledProcessError:
        print('error: not inside a git repository', file=sys.stderr)
        sys.exit(1)


def changed_files(base_ref, root):
    try:
        out = subprocess.check_output(
            ['git', 'diff', '--name-only', f'{base_ref}...HEAD',
             '--', '*.cpp', '*.hpp', '*.h', '*.c'],
            text=True, cwd=root).strip()
        return out.splitlines() if out else []
    except subprocess.CalledProcessError:
        print(f'error: could not diff against "{base_ref}" — '
              'is the ref valid and fetched?', file=sys.stderr)
        sys.exit(1)


def submodule_paths(root):
    # Returns [] if .gitmodules is absent or has no paths — both valid
    try:
        out = subprocess.check_output(
            ['git', 'config', '--file', '.gitmodules',
             '--get-regexp', 'path'],
            text=True, cwd=root).strip()
        return [line.split()[1] for line in out.splitlines()]
    except subprocess.CalledProcessError:
        return []


def build_exclude(root):
    submodules = '|'.join(submodule_paths(root))
    return f'{submodules}|{EXCLUDE_EXTRA}' if submodules else EXCLUDE_EXTRA


def load_db(build_dir):
    db_path = os.path.join(build_dir, 'compile_commands.json')
    if not os.path.isfile(db_path):
        print(f'error: {db_path} not found', file=sys.stderr)
        print('Run "make px4_sitl_default-clang" first to generate '
              'the compilation database', file=sys.stderr)
        sys.exit(1)
    try:
        with open(db_path) as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        print(f'error: compile_commands.json is malformed: {e}', file=sys.stderr)
        sys.exit(1)


def find_tus(changed, db, root):
    db_files = {e['file'] for e in db}
    result = set()
    for f in changed:
        abs_path = os.path.join(root, f)
        ext = os.path.splitext(f)[1]
        if ext in EXTENSIONS_CPP:
            if abs_path in db_files:
                result.add(abs_path)
        elif ext in EXTENSIONS_HDR:
            hdr = os.path.basename(f)
            for e in db:
                try:
                    if hdr in open(e['file']).read():
                        result.add(e['file'])
                except OSError:
                    pass  # file deleted in PR — skip
    return sorted(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('base_ref',
                        help='Git ref to diff against, e.g. origin/main')
    args = parser.parse_args()

    root = repo_root()
    build_dir = os.path.join(root, 'build', 'px4_sitl_default-clang')

    run_tidy = os.path.join(root, 'Tools', 'run-clang-tidy.py')
    if not os.path.isfile(run_tidy):
        print(f'error: {run_tidy} not found', file=sys.stderr)
        sys.exit(1)

    changed = changed_files(args.base_ref, root)
    if not changed:
        print('No C++ files changed — skipping clang-tidy')
        sys.exit(0)

    db = load_db(build_dir)
    tus = find_tus(changed, db, root)

    if not tus:
        print('No matching TUs in compile_commands.json — skipping clang-tidy')
        sys.exit(0)

    print(f'Running clang-tidy on {len(tus)} translation unit(s)')

    result = subprocess.run(
        [sys.executable, run_tidy,
         '-header-filter=.*\\.hpp',
         '-j0',
         f'-exclude={build_exclude(root)}',
         '-p', build_dir] + tus
    )
    sys.exit(result.returncode)


if __name__ == '__main__':
    main()
