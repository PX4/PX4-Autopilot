"""Only people with write access may start a review from a comment, and a
malformed command must never start a default review by accident."""

import os
import tempfile
import unittest
from typing import Any, Dict, List, Optional, Tuple

from ai_review import trigger

ALIASES = {'opus': None, 'sonnet': None}


def event(body: str = '!ai-review', login: str = 'alice',
          user_type: str = 'User', on_pr: bool = True,
          state: str = 'open') -> Dict[str, Any]:
    issue: Dict[str, Any] = {'number': 42, 'state': state}
    if on_pr:
        issue['pull_request'] = {'url': 'x'}
    return {'issue': issue,
            'comment': {'id': 7, 'body': body,
                        'user': {'login': login, 'type': user_type}}}


class FakeApi:

    def __init__(self, permission: Optional[str]) -> None:
        self.permission = permission
        self.calls: List[Tuple[str, str]] = []

    def __call__(self, method: str, path: str,
                 body: Optional[Dict[str, Any]]) -> Any:
        self.calls.append((method, path))
        return {'permission': self.permission}


def decide(ev: Dict[str, Any], permission: Optional[str] = 'write',
           name: str = 'issue_comment') -> trigger.Decision:
    return trigger.decide(name, ev, {}, ALIASES, FakeApi(permission),
                          'PX4/PX4-Autopilot')


class TestParse(unittest.TestCase):

    def test_plain_and_with_arguments(self) -> None:
        self.assertEqual(trigger.parse_command('!ai-review', ALIASES),
                         {'model': 'default', 'effort': 'default'})
        self.assertEqual(
            trigger.parse_command('  !ai-review model=sonnet effort=high\n',
                                  ALIASES),
            {'model': 'sonnet', 'effort': 'high'})

    def test_command_must_be_the_whole_comment(self) -> None:
        for body in ('LGTM\n!ai-review', '!ai-review\nthanks',
                     '> !ai-review\n!ai-review', 'Looks good. !ai-review'):
            with self.subTest(body=body):
                self.assertIsNone(trigger.parse_command(body, ALIASES))

    def test_not_a_command(self) -> None:
        # only the bang form is the command
        for body in ('please run !ai-review', '!ai-reviewer', '',
                     '!ai-review model=fable', '!ai-review effort=ultra',
                     '!ai-review now', '`!ai-review`', '#ai-review',
                     '#!ai-review', '/ai-review', '! ai-review'):
            with self.subTest(body=body):
                self.assertIsNone(trigger.parse_command(body, ALIASES))


class TestDecide(unittest.TestCase):

    def test_write_access_runs(self) -> None:
        for level in ('write', 'maintain', 'admin'):
            with self.subTest(level=level):
                d = decide(event('!ai-review model=sonnet'), level)
                self.assertTrue(d.run)
                self.assertEqual((d.pr, d.model, d.comment_id),
                                 (42, 'sonnet', 7))

    def test_without_write_access_does_not_run(self) -> None:
        for level in ('read', 'triage', 'none', None):
            with self.subTest(level=level):
                d = decide(event(), level)
                self.assertFalse(d.run)
                self.assertIn('write access is required', d.reason)

    def test_permission_is_checked_through_the_api(self) -> None:
        api = FakeApi('write')
        trigger.decide('issue_comment', event(login='bob'), {}, ALIASES, api,
                       'PX4/PX4-Autopilot')
        self.assertEqual(api.calls, [
            ('GET', 'repos/PX4/PX4-Autopilot/collaborators/bob/permission')])

    def test_ignored_events(self) -> None:
        cases = [(event(on_pr=False), 'not on a pull request'),
                 (event(state='closed'), 'not open'),
                 (event(body='nice work'), 'no !ai-review command'),
                 (event(user_type='Bot'), 'bot')]
        for ev, reason in cases:
            with self.subTest(reason=reason):
                d = decide(ev)
                self.assertFalse(d.run)
                self.assertIn(reason, d.reason)

    def test_dispatch_uses_inputs(self) -> None:
        d = trigger.decide('workflow_dispatch', {},
                           {'pr_number': '28817', 'model': 'opus',
                            'effort': 'xhigh'}, ALIASES, FakeApi(None), 'r')
        self.assertTrue(d.run)
        self.assertEqual((d.pr, d.model, d.effort, d.comment_id),
                         (28817, 'opus', 'xhigh', 0))

    def test_outputs_written_for_github(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'out')
            trigger.write_outputs(decide(event()), path)
            text = open(path).read()
        self.assertIn('run=true\n', text)
        self.assertIn('pr=42\n', text)


if __name__ == '__main__':
    unittest.main()
