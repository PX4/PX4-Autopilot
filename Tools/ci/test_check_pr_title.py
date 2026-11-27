"""Exercise the PR title check on backport titles."""

import contextlib
import io
import unittest

from check_pr_title import check_title, format_markdown, suggest_title


def passes(title, base_ref=None):
    with contextlib.redirect_stderr(io.StringIO()):
        return check_title(title, base_ref)


class BackportTitleTest(unittest.TestCase):
    def test_backport_tag_in_front_of_a_valid_header_passes(self):
        for title in (
            'fix(mavlink): let a UDP client reconnect from a new port',
            '[BACKPORT 1.18] fix(mavlink): let a UDP client reconnect from a new port',
            '[BACKPORT v1.18] feat(boards/ark_fpv)!: add the LSM6DSV32X revision',
            '[Backport 1.17] docs(docs): dynamically fetched navbar',
            '[BACKPORT] fix(boards/matek-h743): correct board ID',
        ):
            with self.subTest(title=title):
                self.assertTrue(passes(title))

    def test_backport_tag_does_not_exempt_the_header(self):
        for title in (
            '[BACKPORT 1.18] Fix the UDP client reconnect',
            '[BACKPORT 1.18] mavlink: let a UDP client reconnect',
            '[BACKPORT 1.18]',
            '[Docs] fix(mavlink): let a UDP client reconnect from a new port',
        ):
            with self.subTest(title=title):
                self.assertFalse(passes(title))

    def test_backport_tag_needs_a_release_base_branch(self):
        title = '[BACKPORT 1.18] fix(mavlink): let a UDP client reconnect from a new port'
        self.assertTrue(passes(title, 'release/1.18'))
        self.assertFalse(passes(title, 'main'))
        self.assertIn('targets `main`', format_markdown(title, 'main'))
        self.assertTrue(passes('fix(mavlink): let a UDP client reconnect from a new port', 'main'))

    def test_suggestion_keeps_the_backport_tag(self):
        self.assertEqual(
            suggest_title('[BACKPORT 1.18] mavlink: let a UDP client reconnect'),
            '[BACKPORT 1.18] feat(mavlink): let a UDP client reconnect')


if __name__ == '__main__':
    unittest.main()
