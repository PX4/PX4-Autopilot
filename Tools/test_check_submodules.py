#!/usr/bin/env python3
"""Tests for check_submodules.sh against throwaway git repositories."""

import os
import shutil
import subprocess
import tempfile
import unittest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'check_submodules.sh')

GIT_ENV = {
    'GIT_AUTHOR_NAME': 'test', 'GIT_AUTHOR_EMAIL': 'test@example.com',
    'GIT_COMMITTER_NAME': 'test', 'GIT_COMMITTER_EMAIL': 'test@example.com',
    # submodules are cloned from local paths, and test commits are never signed
    'GIT_CONFIG_COUNT': '2',
    'GIT_CONFIG_KEY_0': 'protocol.file.allow', 'GIT_CONFIG_VALUE_0': 'always',
    'GIT_CONFIG_KEY_1': 'commit.gpgsign', 'GIT_CONFIG_VALUE_1': 'false',
}


class CheckSubmodulesTest(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)
        self.env = {k: v for k, v in os.environ.items()
                    if k not in ('CI', 'GIT_SUBMODULES_ARE_EVIL') and not k.startswith('GIT_')}
        self.env.update(GIT_ENV)

        # lib, with a nested submodule inner, is a submodule of the checkout at ext/lib
        inner = self.repo('inner')
        lib = self.repo('lib')
        self.git(lib, 'submodule', 'add', '-q', inner, 'inner')
        self.git(lib, 'commit', '-qm', 'add inner')
        top = self.repo('top')
        self.git(top, 'submodule', 'add', '-q', lib, 'ext/lib')
        self.git(top, 'commit', '-qm', 'add lib')

        self.checkout = os.path.join(self.tmp, 'checkout')
        self.git(self.tmp, 'clone', '-q', top, self.checkout)
        os.mkdir(os.path.join(self.checkout, 'Tools'))
        shutil.copy(SCRIPT, os.path.join(self.checkout, 'Tools'))
        self.lib = os.path.join(self.checkout, 'ext', 'lib')
        self.inner = os.path.join(self.lib, 'inner')

    def repo(self, name):
        path = os.path.join(self.tmp, name)
        self.git(self.tmp, 'init', '-q', name)
        self.git(path, 'commit', '-q', '--allow-empty', '-m', name)
        return path

    def git(self, cwd, *args):
        return subprocess.run(['git', *args], cwd=cwd, env=self.env, check=True,
                              capture_output=True, text=True).stdout.strip()

    def check(self, **env):
        return subprocess.run(['Tools/check_submodules.sh'], cwd=self.checkout,
                              env={**self.env, **env}, capture_output=True, text=True)

    def fetched(self, path):
        return os.path.exists(os.path.join(path, '.git'))

    def change(self, path):
        """Commit in a submodule, as a developer testing a submodule change would."""
        self.git(path, 'commit', '-q', '--allow-empty', '-m', 'change')
        return self.git(path, 'rev-parse', 'HEAD')

    def test_missing_submodules_are_fetched(self):
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertTrue(self.fetched(self.lib))
        self.assertTrue(self.fetched(self.inner))

    def test_recorded_commits_need_nothing(self):
        self.check()
        result = self.check(CI='true')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(result.stdout, '')

    def test_missing_nested_submodule_is_fetched(self):
        self.git(self.checkout, 'submodule', 'update', '--init', '-q', 'ext/lib')
        self.assertFalse(self.fetched(self.inner))
        self.assertEqual(self.check().returncode, 0)
        self.assertTrue(self.fetched(self.inner))

    def test_changed_submodule_warns_and_is_kept(self):
        self.check()
        head = self.change(self.lib)
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('Warning', result.stdout)
        self.assertIn('ext/lib', result.stdout)
        self.assertEqual(self.git(self.lib, 'rev-parse', 'HEAD'), head)

    def test_changed_submodule_fails_in_ci(self):
        self.check()
        head = self.change(self.inner)
        result = self.check(CI='true')
        self.assertEqual(result.returncode, 1)
        self.assertIn('ext/lib/inner', result.stdout)
        self.assertEqual(self.git(self.inner, 'rev-parse', 'HEAD'), head)

    def test_missing_nested_submodule_of_changed_submodule(self):
        self.git(self.checkout, 'submodule', 'update', '--init', '-q', 'ext/lib')
        head = self.change(self.lib)
        self.assertEqual(self.check().returncode, 0)
        self.assertTrue(self.fetched(self.inner))
        self.assertEqual(self.git(self.lib, 'rev-parse', 'HEAD'), head)

    def test_git_submodules_are_evil_skips_everything(self):
        result = self.check(GIT_SUBMODULES_ARE_EVIL='1')
        self.assertEqual(result.returncode, 0)
        self.assertFalse(self.fetched(self.lib))


if __name__ == '__main__':
    unittest.main()
