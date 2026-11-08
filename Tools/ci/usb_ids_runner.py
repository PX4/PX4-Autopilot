#!/usr/bin/env python3
"""Check board defconfigs against the Dronecode USB ID registry
(https://github.com/Dronecode/usb-ids).

Meant for the usb_ids.yml workflow; run from the repository root.
A pull request checks the defconfigs it adds or modifies, or every board
defconfig when it changes the USB ID tooling itself. A manual run checks
every board defconfig.

On a pull request, actions/checkout checks out the test merge commit,
whose first parent is the base branch; the checkout needs fetch-depth 2
so the PR's changes can be diffed locally against it.
"""

import glob
import os
import re
import subprocess
import sys
from typing import List, Mapping, Tuple

import check_usb_ids

DEFCONFIG_RE = re.compile(r'^boards/[^/]+/[^/]+/nuttx-config/.+/defconfig$')
# Keep in sync with the pull_request paths filter in usb_ids.yml
TOOLING = frozenset((
    '.github/workflows/usb_ids.yml',
    'Tools/ci/check_usb_ids.py',
    'Tools/ci/test_check_usb_ids.py',
    'Tools/ci/test_usb_ids_runner.py',
    'Tools/ci/usb_ids_runner.py',
))


def all_defconfigs() -> List[str]:
    return sorted(glob.glob('boards/*/*/nuttx-config/*/defconfig'))


def pr_files() -> List[str]:
    """Files the checked-out PR merge commit adds or modifies."""
    diff = subprocess.run(
        ['git', 'diff', '--name-only', '--diff-filter=d', 'HEAD^1', 'HEAD'],
        capture_output=True, text=True)
    if diff.returncode != 0:
        sys.exit(f'error: cannot diff the PR merge commit against its base '
                 f'(is fetch-depth 2?): {diff.stderr.strip()}')
    return diff.stdout.splitlines()


def changed_files(env: Mapping[str, str]) -> Tuple[List[str], bool]:
    """Return (paths, full_tree) for the triggering event."""
    if env.get('GITHUB_EVENT_NAME') != 'pull_request':
        return all_defconfigs(), True

    files = pr_files()
    if TOOLING.intersection(files):
        # a change to the checker itself is proven against every board
        return all_defconfigs(), True
    return files, False


def main() -> int:
    paths, full_tree = changed_files(os.environ)
    defconfigs = [p for p in paths if DEFCONFIG_RE.match(p)]
    if not defconfigs:
        print('No board defconfig changes, nothing to check.')
        return 0

    if full_tree:
        print(f'Checking all {len(defconfigs)} board defconfigs.')
    else:
        print('Changed defconfigs:')
        print('\n'.join(defconfigs))
    return check_usb_ids.cmd_check(check_usb_ids.load_registry(), defconfigs)


if __name__ == '__main__':
    sys.exit(main())
