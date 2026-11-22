"""Check which defconfigs usb_ids_runner.py hands to the checker.

A selection bug fails open: if a PR resolves to the wrong file list, a
changed board is never checked and the job still goes green. PR diffs
run against a real throwaway git repo shaped like the checkout
actions/checkout makes for a pull request: a merge commit whose first
parent is the base branch.
"""

import contextlib
import io
import os
from pathlib import Path
import subprocess
import tempfile
from typing import List, Tuple
import unittest
from unittest import mock

import yaml

import check_usb_ids
import usb_ids_runner
from usb_ids_runner import changed_files

WORKFLOW = (Path(__file__).resolve().parents[2]
            / '.github/workflows/usb_ids.yml')
ALL = ['boards/px4/fmu-v6xrt/nuttx-config/nsh/defconfig']
PR_ENV = {'GITHUB_EVENT_NAME': 'pull_request'}
KEPT = 'boards/siyi/n7/nuttx-config/nsh/defconfig'
GONE = 'boards/old/fc/nuttx-config/nsh/defconfig'


class ChangedFilesTest(unittest.TestCase):
    def setUp(self) -> None:
        patcher = mock.patch.object(usb_ids_runner, 'all_defconfigs',
                                    return_value=ALL)
        patcher.start()
        self.addCleanup(patcher.stop)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.repo = Path(tmp.name)
        cwd = os.getcwd()
        os.chdir(self.repo)
        self.addCleanup(os.chdir, cwd)
        self.git('init', '-q', '-b', 'base')
        self.write(GONE)
        self.write('README.md')
        self.git('add', '.')
        self.git('commit', '-q', '-m', 'base')

    def git(self, *args: str) -> None:
        subprocess.run(['git', '-c', 'user.name=t', '-c', 'user.email=t@t',
                        *args], check=True, capture_output=True)

    def write(self, path: str) -> None:
        (self.repo / path).parent.mkdir(parents=True, exist_ok=True)
        (self.repo / path).write_text(path)

    def merge_pr(self, *files: str) -> None:
        """Commit files on a PR branch that also removes GONE, then merge
        it into base with a merge commit, as the PR checkout does."""
        self.git('checkout', '-q', '-b', 'pr')
        for f in files:
            self.write(f)
        self.git('rm', '-q', GONE)
        self.git('add', '.')
        self.git('commit', '-q', '-m', 'pr')
        self.git('checkout', '-q', 'base')
        self.write('base-moved-on.txt')
        self.git('add', '.')
        self.git('commit', '-q', '-m', 'base moves on')
        self.git('merge', '-q', '--no-ff', '-m', 'merge', 'pr')

    def test_manual_run_checks_the_full_tree(self) -> None:
        env = {'GITHUB_EVENT_NAME': 'workflow_dispatch'}
        self.assertEqual(changed_files(env), (ALL, True))

    def test_pull_request_diffs_against_base_without_removed(self) -> None:
        # base's own later commits and the PR's deletion are left out
        self.merge_pr(KEPT)
        self.assertEqual(changed_files(PR_ENV), ([KEPT], False))

    def test_tooling_change_checks_the_full_tree(self) -> None:
        self.merge_pr(KEPT, 'Tools/ci/check_usb_ids.py')
        self.assertEqual(changed_files(PR_ENV), (ALL, True))

    def test_shallow_checkout_without_base_fails_loudly(self) -> None:
        # HEAD has no parent here, as with the default fetch-depth 1
        with self.assertRaises(SystemExit) as cm:
            changed_files(PR_ENV)
        self.assertIn('fetch-depth', str(cm.exception.code))

    def test_tooling_matches_the_workflow_paths_filter(self) -> None:
        # a file missing from the filter never triggers the workflow, and
        # one missing from TOOLING never forces the full-tree check
        on = yaml.safe_load(WORKFLOW.read_text())[True]
        paths = set(on['pull_request']['paths'])
        self.assertEqual(paths - {'boards/*/*/nuttx-config/*/defconfig'},
                         set(usb_ids_runner.TOOLING))


class MainTest(unittest.TestCase):
    def run_main(self, paths: List[str]
                 ) -> Tuple[int, str, mock.MagicMock, mock.MagicMock]:
        out = io.StringIO()
        env = {'GITHUB_EVENT_NAME': 'workflow_dispatch'}
        with mock.patch.dict(os.environ, env, clear=True), \
                mock.patch.object(usb_ids_runner, 'changed_files',
                                  return_value=(paths, False)), \
                mock.patch.object(check_usb_ids, 'load_registry') as load, \
                mock.patch.object(check_usb_ids, 'cmd_check',
                                  return_value=0) as check, \
                contextlib.redirect_stdout(out):
            rc = usb_ids_runner.main()
        return rc, out.getvalue(), load, check

    def test_only_board_defconfigs_reach_the_checker(self) -> None:
        wanted = ['boards/px4/fmu-v6xrt/nuttx-config/nsh/defconfig',
                  'boards/siyi/n7/nuttx-config/bootloader/defconfig']
        ignored = ['boards/px4/fmu-v6xrt/default.px4board',
                   'boards/px4/fmu-v6xrt/nuttx-config/nsh/defconfig.orig',
                   'boards/px4/defconfig',
                   'platforms/nuttx/NuttX/nuttx/boards/arm/stm32/x/configs/'
                   'nsh/defconfig',
                   'Tools/ci/usb_ids_runner.py']
        rc, _, _, check = self.run_main(ignored + wanted)
        self.assertEqual(rc, 0)
        self.assertEqual(check.call_args.args[1], wanted)

    def test_no_defconfigs_exits_before_fetching_the_registry(self) -> None:
        rc, out, load, check = self.run_main(['Tools/ci/usb_ids_runner.py'])
        self.assertEqual(rc, 0)
        self.assertIn('No board defconfig changes', out)
        load.assert_not_called()
        check.assert_not_called()


if __name__ == '__main__':
    unittest.main()
