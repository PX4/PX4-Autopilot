"""The system prompt must carry the repository rules that apply to the
changed paths, and the offline render path must produce a postable
artifact from saved findings without calling a model."""

import json
import re
import tempfile
import unittest
from pathlib import Path

from ai_review import __main__ as cli
from ai_review import agent, pipeline

ROOT = Path(__file__).resolve().parents[3]
DIFF = """\
diff --git a/src/modules/ekf2/a.cpp b/src/modules/ekf2/a.cpp
--- a/src/modules/ekf2/a.cpp
+++ b/src/modules/ekf2/a.cpp
@@ -1,2 +1,2 @@
 void f() {
-  x = 1;
+  x = 2;
"""


class TestWorkflow(unittest.TestCase):

    def test_workspace_paths_are_well_formed(self) -> None:
        # a bulk rename once turned "$GITHUB_WORKSPACE/ai-review-work" into
        # "$GITHUB_WORKSPACE!ai-review-work", silently losing the report
        text = (ROOT / '.github/workflows/pr-ai-review.yml').read_text()
        uses = re.findall(r'\$GITHUB_WORKSPACE(.)', text)
        self.assertTrue(uses)
        self.assertEqual(set(uses) - {'/', '"'}, set())


class TestSizeScaling(unittest.TestCase):

    def test_no_size_skips_only_a_larger_budget(self) -> None:
        base = agent.AgentConfig('m', 'xhigh', 80, 1500)
        small = pipeline.scaled_for_size(base, 30)
        huge = pipeline.scaled_for_size(base, 20261)
        self.assertEqual((small.max_turns, small.timeout_s), (80, 1500))
        self.assertGreater(huge.max_turns, small.max_turns)
        self.assertGreater(huge.timeout_s, small.timeout_s)
        sizes = [10, 300, 301, 1500, 1501, 6000, 6001, 50000]
        turns = [pipeline.scaled_for_size(base, n).max_turns for n in sizes]
        self.assertEqual(turns, sorted(turns))


class TestSystemPrompt(unittest.TestCase):

    def test_matches_instructions_by_apply_to(self) -> None:
        names = [p.name for p in pipeline.matching_instructions(
            ROOT, ['src/modules/ekf2/EKF/ekf.cpp'])]
        self.assertIn('estimation.instructions.md', names)
        self.assertNotIn('docs.en.instructions.md', names)

    def test_sandbox_note_only_when_enabled(self) -> None:
        self.assertNotIn('px4-sandbox-python',
                         pipeline.system_prompt(ROOT, []))
        self.assertIn('px4-sandbox-python',
                      pipeline.system_prompt(ROOT, [], sandbox=True))
        self.assertNotIn('px4-sandbox-python', pipeline.validator_prompt())
        self.assertIn('px4-sandbox-python',
                      pipeline.validator_prompt(sandbox=True))

    def test_contribution_requirements_are_ci_only(self) -> None:
        # the interactive skill reads review-criteria.md and must stay lean
        shared = (ROOT / '.agents/skills/review-pr/review-criteria.md'
                  ).read_text()
        self.assertNotIn('## Contribution requirements', shared)
        self.assertIn('Contribution requirements (CI review only)',
                      pipeline.system_prompt(ROOT, []))

    def test_prompt_includes_criteria_and_agents(self) -> None:
        text = pipeline.system_prompt(ROOT, ['docs/en/index.md'])
        self.assertIn('PX4 review criteria', text)
        self.assertIn('docs.en.instructions.md', text)
        self.assertIn('Automated PX4 pull request review', text)


class TestRender(unittest.TestCase):

    def test_render_saved_findings(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            t = Path(tmp)
            checkout = t / 'pr'
            src = checkout / 'src/modules/ekf2'
            src.mkdir(parents=True)
            (src / 'a.cpp').write_text('void f() {\n  x = 2;\n')
            (t / 'pr.diff').write_text(DIFF)
            (t / 'findings.json').write_text(json.dumps({
                'summary': 'Changes x.',
                'pairs': [{
                    'finding': {
                        'path': 'src/modules/ekf2/a.cpp', 'line': 2,
                        'start_line': None, 'severity': 'blocker',
                        'title': 'Wrong value', 'body': 'b', 'trigger': 't',
                        'suggestion': 'Restore 1.', 'replacement': '  x = 1;',
                        'uncertainty': '', 'rule': None},
                    'verdict': {'keep': True, 'confidence': 'high',
                                'reason': 'traced'}}]}))
            rc = cli.main([
                'render', '--pr', '5', '--trusted-root', str(ROOT),
                '--checkout', str(checkout), '--work-dir', str(t / 'w'),
                '--out-dir', str(t / 'out'), '--findings',
                str(t / 'findings.json'), '--diff', str(t / 'pr.diff'),
                '--head-sha', 'c' * 40])
            self.assertEqual(rc, 0)
            comments = json.loads((t / 'out/comments.json').read_text())
            self.assertEqual(len(comments), 1)
            self.assertIn('```suggestion', comments[0]['body'])
            manifest = json.loads((t / 'out/manifest.json').read_text())
            self.assertIn('blocking issues', manifest['summary'])


if __name__ == '__main__':
    unittest.main()
