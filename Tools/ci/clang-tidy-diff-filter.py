#!/usr/bin/env python3
"""
Filter a git diff for consumption by clang-tidy-diff.

Produces a unified diff containing only files that clang-tidy can
actually analyze against the current compilation database:

  - C/C++ source files (.c, .cpp, .cc, .cxx, .m, .mm) must be present
    in compile_commands.json. Files absent from the database are test
    files, excluded code, or platform-specific sources that were not
    compiled. Feeding them to clang-tidy-diff produces spurious
    "header not found" errors (gtest/gtest.h in particular).

  - Header files (.h, .hpp, .hxx) always pass through. clang-tidy
    analyzes header changes via the TUs that include them; there is
    no separate TU for a header to match against the database.

  - All other files (CMakeLists.txt, .yml, .md, etc.) are dropped.

Output is a unified diff suitable for piping into clang-tidy-diff.py.
If nothing remains, the output file is empty.

Used by .github/workflows/clang-tidy.yml as a pre-filter for the
`pr-review` artifact producer. Python stdlib only.
"""

import argparse
import json
import os
import subprocess
import sys


SOURCE_EXTS = {'.c', '.cpp', '.cc', '.cxx', '.m', '.mm'}
HEADER_EXTS = {'.h', '.hpp', '.hxx'}


def load_db_files(build_dir):
    """Return the set of source paths (repo-relative) in compile_commands.json."""
    path = os.path.join(build_dir, 'compile_commands.json')
    with open(path) as f:
        db = json.load(f)
    root = os.path.abspath('.')
    prefix = root + os.sep
    paths = set()
    for entry in db:
        p = entry.get('file', '')
        if p.startswith(prefix):
            paths.add(p[len(prefix):])
        else:
            # Relative or external path; record as-is
            paths.add(p)
    return paths


def changed_files(base_ref):
    out = subprocess.check_output(
        ['git', 'diff', '--name-only', '{}...HEAD'.format(base_ref)],
        text=True,
    )
    return [line.strip() for line in out.splitlines() if line.strip()]


def keep_file(path, db_files):
    """Decide whether to keep this path in the filtered diff."""
    ext = os.path.splitext(path)[1].lower()
    if ext in HEADER_EXTS:
        return True
    if ext in SOURCE_EXTS:
        return path in db_files
    return False


def filtered_diff(base_ref, keep_paths):
    if not keep_paths:
        return ''
    cmd = ['git', 'diff', '-U0', '{}...HEAD'.format(base_ref), '--'] + sorted(keep_paths)
    return subprocess.check_output(cmd, text=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', required=True,
                        help='CMake build dir containing compile_commands.json')
    parser.add_argument('--base-ref', required=True,
                        help='Git ref to diff against (e.g. origin/main)')
    parser.add_argument('--out', required=True,
                        help='Output path for the filtered unified diff')
    args = parser.parse_args()

    db_files = load_db_files(args.build_dir)
    changed = changed_files(args.base_ref)

    keep = [p for p in changed if keep_file(p, db_files)]
    dropped = [p for p in changed if p not in keep]

    print('clang-tidy-diff-filter: kept {} of {} changed files'.format(
        len(keep), len(changed)))
    if dropped:
        print('  dropped (not in compile_commands.json or not source/header):')
        for p in dropped:
            print('    {}'.format(p))

    diff = filtered_diff(args.base_ref, keep)
    with open(args.out, 'w') as f:
        f.write(diff)
    return 0


if __name__ == '__main__':
    sys.exit(main())
