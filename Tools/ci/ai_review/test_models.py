"""Upgrading a model must be a one-line edit to models.json that either
works or fails loudly, never a silent fallback to something else."""

import copy
import json
import subprocess
import unittest
from typing import Any, Dict, List, Sequence

from ai_review import models

VALID: Dict[str, Any] = {
    'models': {
        'opus': {'bedrock_profile': 'global.anthropic.claude-opus-5-5',
                 'label': 'Claude Opus 5.5'},
        'sonnet': {'bedrock_profile': 'global.anthropic.claude-sonnet-5-5',
                   'label': 'Claude Sonnet 5.5'},
    },
    'defaults': {'reviewer': {'model': 'opus', 'effort': 'xhigh'},
                 'validator': {'model': 'opus', 'effort': 'high'}},
}


def variant(**paths: Any) -> Dict[str, Any]:
    data = copy.deepcopy(VALID)
    for dotted, value in paths.items():
        node = data
        keys = dotted.split('__')
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return data


class TestRegistry(unittest.TestCase):

    def test_shipped_registry_is_valid(self) -> None:
        reg = models.load()
        self.assertIn(reg.defaults['reviewer'].model, reg.models)

    def test_upgrade_is_a_profile_change(self) -> None:
        reg = models.parse(variant(
            models__opus__bedrock_profile='global.anthropic.claude-opus-5-6',
            models__opus__label='Claude Opus 5.6'))
        self.assertEqual(reg.get('opus').label, 'Claude Opus 5.6')

    def test_rejects_profiles_outside_allowed_families(self) -> None:
        for profile in ('us.anthropic.claude-opus-5-5',
                        'global.anthropic.claude-fable-5-1',
                        'anthropic.claude-opus-5-5', ''):
            with self.subTest(profile=profile):
                with self.assertRaises(models.RegistryError):
                    models.parse(variant(
                        models__opus__bedrock_profile=profile))

    def test_defaults_must_name_known_alias_and_effort(self) -> None:
        for bad in (variant(defaults__reviewer__model='fable'),
                    variant(defaults__validator__effort='ultra')):
            with self.subTest(bad=json.dumps(bad['defaults'])):
                with self.assertRaises(models.RegistryError):
                    models.parse(bad)

    def test_unknown_alias_lists_known_ones(self) -> None:
        with self.assertRaisesRegex(models.RegistryError, 'opus, sonnet'):
            models.parse(VALID).get('haiku')


class TestCliDefaults(unittest.TestCase):

    def test_default_resolves_from_registry(self) -> None:
        from ai_review import __main__ as cli
        args = cli.parser().parse_args([
            'run', '--pr', '1', '--trusted-root', '.', '--checkout', '.',
            '--model', 'default', '--effort', 'default'])
        s = cli._settings(args)
        reg = models.load()
        rev = reg.defaults['reviewer']
        self.assertEqual(s.reviewer.model,
                         reg.get(rev.model).bedrock_profile)
        self.assertEqual(s.reviewer.effort, rev.effort)


class TestCheck(unittest.TestCase):

    def test_reports_each_failing_model(self) -> None:
        calls: List[List[str]] = []

        def runner(cmd: Sequence[str]) -> 'subprocess.CompletedProcess[str]':
            calls.append(list(cmd))
            bad = 'sonnet' in ' '.join(cmd)
            return subprocess.CompletedProcess(
                list(cmd), 1 if bad else 0, 'OK',
                'AccessDeniedException' if bad else '')

        failures = models.check(models.parse(VALID), 'us-west-2', runner)
        self.assertEqual(len(calls), 2)
        self.assertEqual(len(failures), 1)
        self.assertIn('sonnet', failures[0])
        self.assertIn('AccessDenied', failures[0])


if __name__ == '__main__':
    unittest.main()
