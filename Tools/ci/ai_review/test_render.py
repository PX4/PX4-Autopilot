"""The artifact must satisfy pr-review-poster.py's contract, and no
credential may ever be written into it."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict

from ai_review import render, route
from ai_review.findings import (CHECKLIST_ITEMS, ChecklistEntry, Finding,
                                Verdict)

POSTER = Path(__file__).resolve().parents[1] / 'pr-review-poster.py'


def placed(**over: Any) -> route.Placed:
    data: Dict[str, Any] = dict(
        path='src/a.cpp', line=3, start_line=2, severity='blocker',
        title='Overflow', body='b', trigger='t', suggestion='s',
        replacement=None, uncertainty='', rule=None)
    data.update(over)
    return route.Placed(finding=Finding(**data),
                        verdict=Verdict(True, 'high', 'r'),
                        replacement='  x = 1;')


class TestRender(unittest.TestCase):

    def test_comment_has_suggestion_block_and_tag(self) -> None:
        body = render.comment_body(placed())
        self.assertIn('```suggestion\n  x = 1;\n```', body)
        self.assertTrue(body.rstrip().endswith(render.FINDING_TAG))

    def test_multi_line_comment_fields(self) -> None:
        routed = route.Routed(inline=[placed()])
        c = render.comments(routed)[0]
        self.assertEqual((c['start_line'], c['line'], c['side']),
                         (2, 3, 'RIGHT'))

    def test_summary_collapses_lower_confidence(self) -> None:
        low = placed(severity='concern', start_line=None)
        routed = route.Routed(collapsed=[low])
        text = render.summary(routed, 'Risky.', 'Claude Opus 5.5')
        self.assertIn('<details>', text)
        self.assertIn(route.VERDICT_CLEAN, text)

    def test_summary_lists_before_merge_and_checklist(self) -> None:
        pr = placed(path=None, line=None, start_line=None,
                    severity='concern', title='No test evidence')
        routed = route.Routed(before_merge=[pr])
        checklist = {k: ChecklistEntry('ok', '') for k in CHECKLIST_ITEMS}
        checklist['tests'] = ChecklistEntry('gap', 'none')
        text = render.summary(routed, '', 'm', checklist)
        self.assertIn('**Before merge**', text)
        self.assertIn('No test evidence', text)
        self.assertIn('⚠️ test evidence', text)
        self.assertNotIn('<details>', text)

    def test_summary_is_truncated_under_limit(self) -> None:
        big = placed(body='x' * 3000, severity='concern')
        routed = route.Routed(collapsed=[big] * 40)
        text = render.summary(routed, '', 'm')
        self.assertLessEqual(len(text.encode('utf-8')), render.SUMMARY_LIMIT)

    def test_artifact_passes_poster_validation(self) -> None:
        spec = importlib.util.spec_from_file_location('poster', POSTER)
        assert spec and spec.loader
        poster = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(poster)
        with tempfile.TemporaryDirectory() as tmp:
            render.write_artifact(Path(tmp), 123, 'a' * 40,
                                  route.Routed(inline=[placed()]), 's', 'm')
            result = poster.validate_manifest(tmp)
            self.assertEqual(result['pr_number'], 123)
            comments = json.loads((Path(tmp) / 'comments.json').read_text())
            self.assertEqual(len(comments), 1)


class TestCostLine(unittest.TestCase):

    def test_cost_in_review_footer(self) -> None:
        usage = {'cost_usd': 7.4239, 'calls': 12, 'wall_seconds': 1290.0}
        text = render.summary(route.Routed(), '', 'Claude Opus 5.5',
                              usage=usage)
        self.assertIn('Cost: $7.42 · Claude Opus 5.5 · 12 model call(s) · '
                      '22 min', text)

    def test_no_usage_no_cost_line(self) -> None:
        self.assertEqual(render.cost_line(None, 'm'), '')
        self.assertNotIn('Cost:', render.summary(route.Routed(), '', 'm'))

    def test_refusal_shows_what_was_spent(self) -> None:
        from ai_review import agent
        e = agent.AgentRefusal('m', 'declined', 'cyber', 'req')
        text = render.refusal_summary(e, 'm', {'cost_usd': 0.01,
                                               'calls': 0})
        self.assertIn('Cost: $0.01', text)


class TestLeaks(unittest.TestCase):

    def test_detects_secret(self) -> None:
        self.assertEqual(render.find_leaks(['a ASIAEXAMPLEKEY1234 b'],
                                           ['ASIAEXAMPLEKEY1234']),
                         ['secret #0'])

    def test_ignores_empty_and_short_values(self) -> None:
        self.assertEqual(render.find_leaks(['abc'], ['', 'abc']), [])


if __name__ == '__main__':
    unittest.main()
