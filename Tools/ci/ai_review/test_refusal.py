"""A refusal must never look like a review, and must carry the provider's
exact reason."""

import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from typing import List, Mapping

from ai_review import __main__ as cli
from ai_review import agent, render

POSTER = Path(__file__).resolve().parents[1] / 'pr-review-poster.py'
REASON = ("API Error: Opus 5.5's safeguards flagged this session "
          "(https://www.anthropic.com/legal/aup).\n\nDetails: `[cyber]`\n\n"
          "Request ID: c7df80aa-04a2-4041-9beb-1e5597ce5023")
REFUSED = {'type': 'result', 'is_error': True, 'stop_reason': 'refusal',
           'result': REASON, 'total_cost_usd': 0.01}
CFG = agent.AgentConfig(model='global.anthropic.claude-opus-5-5',
                        effort='xhigh', max_turns=10, timeout_s=60)


def runner(stdout: str, rc: int) -> agent.Runner:
    def run(cmd: List[str], stdin: str, cwd: Path, env: Mapping[str, str],
            timeout: float) -> 'subprocess.CompletedProcess[str]':
        return subprocess.CompletedProcess(cmd, rc, stdout, '')
    return run


class TestRefusal(unittest.TestCase):

    def test_detected_with_exact_reason_category_and_request_id(
            self) -> None:
        for rc in (0, 1):
            with self.subTest(rc=rc):
                with self.assertRaises(agent.AgentRefusal) as ctx:
                    agent.run(CFG, Path('/p'), {}, 't', 'c', Path('.'),
                              runner=runner(json.dumps(REFUSED), rc))
                e = ctx.exception
                self.assertEqual(e.reason, REASON)
                self.assertEqual(e.category, 'cyber')
                self.assertEqual(e.request_id,
                                 'c7df80aa-04a2-4041-9beb-1e5597ce5023')

    def test_stop_details_category_wins(self) -> None:
        data = dict(REFUSED, stop_details={'category': 'bio'})
        refusal = agent.refusal_from(data, 'm')
        assert refusal is not None
        self.assertEqual(refusal.category, 'bio')

    def test_not_a_refusal(self) -> None:
        self.assertIsNone(agent.refusal_from({'stop_reason': 'end_turn'}, 'm'))

    def test_notice_says_not_reviewed_and_passes_reason_verbatim(
            self) -> None:
        e = agent.refusal_from(REFUSED, 'Claude Opus 5.5')
        assert e is not None
        text = render.refusal_summary(e, 'Claude Opus 5.5')
        self.assertIn('Automated review not done', text)
        self.assertIn('no findings were produced', text)
        self.assertIn('`cyber`', text)
        self.assertIn('c7df80aa-04a2-4041-9beb-1e5597ce5023', text)
        self.assertIn(REASON, text)

    def test_refusal_artifact_has_no_findings_and_passes_poster(
            self) -> None:
        e = agent.refusal_from(REFUSED, 'm')
        assert e is not None
        spec = importlib.util.spec_from_file_location('poster', POSTER)
        assert spec and spec.loader
        poster = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(poster)
        with tempfile.TemporaryDirectory() as tmp:
            render.write_refusal(Path(tmp), 7, 'a' * 40, e, 'm')
            result = poster.validate_manifest(tmp)
            self.assertEqual(result['comments'], [])
            self.assertIn('not done', result['summary'])

    def test_annotation_is_escaped_onto_one_line(self) -> None:
        line = cli._annotation('warning', 'AI review, not done', 'a\nb%')
        self.assertEqual(line, '::warning title=AI review%2C not done::'
                               'a%0Ab%25')


if __name__ == '__main__':
    unittest.main()
