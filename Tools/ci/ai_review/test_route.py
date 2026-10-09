"""Routing is the confidence policy: these tests pin which findings may
appear inline, which are listed in the review body, which stay in the
report, and when a one-click suggestion block is offered."""

import unittest
from typing import Any, Dict, List

from ai_review import diff, route
from ai_review.findings import Finding, Verdict

DIFF = """\
diff --git a/src/a.cpp b/src/a.cpp
--- a/src/a.cpp
+++ b/src/a.cpp
@@ -1,3 +1,4 @@
 void f() {
-  x = 1;
+  x = 2;
+  y = 3;
 }
"""
HEAD = {'src/a.cpp': ['void f() {', '  x = 2;', '  y = 3;', '}', '']}


def make(**over: Any) -> Finding:
    data: Dict[str, Any] = dict(
        path='src/a.cpp', line=2, start_line=None, severity='concern',
        kind='code', title='t', comment='c', body='b', trigger='when',
        suggestion='Use 1.',
        replacement=None, uncertainty='', rule=None)
    data.update(over)
    return Finding(**data)


def keep(confidence: str = 'high') -> Verdict:
    return Verdict(keep=True, confidence=confidence, reason='checked')


class TestRoute(unittest.TestCase):

    def run_route(self, pairs: List[Any]) -> route.Routed:
        return route.route(pairs, diff.parse(DIFF), HEAD)

    def test_high_confidence_in_diff_goes_inline(self) -> None:
        r = self.run_route([(make(), keep())])
        self.assertEqual(len(r.inline), 1)
        self.assertEqual(r.verdict, route.VERDICT_CHANGES)

    def test_blocker_sets_blocking_verdict(self) -> None:
        r = self.run_route([(make(severity='blocker'), keep())])
        self.assertEqual(r.verdict, route.VERDICT_BLOCKING)

    def test_rejected_by_validator_is_dropped(self) -> None:
        r = self.run_route([(make(), Verdict(False, 'high', 'wrong'))])
        self.assertEqual((len(r.inline), len(r.collapsed)), (0, 0))
        self.assertEqual(len(r.dropped), 1)

    def test_medium_confidence_is_collapsed(self) -> None:
        r = self.run_route([(make(), keep('medium'))])
        self.assertEqual(len(r.inline), 0)
        self.assertEqual(len(r.collapsed), 1)
        self.assertEqual(r.verdict, route.VERDICT_CLEAN)

    def test_nits_are_never_posted(self) -> None:
        r = self.run_route([(make(severity='nit'), keep()),
                            (make(path=None, line=None, severity='nit'),
                             keep()),
                            (make(kind='process', severity='nit'), keep())])
        self.assertEqual((len(r.inline), len(r.before_merge),
                          len(r.collapsed), len(r.process)), (0, 0, 0, 0))
        self.assertEqual([p.note for p in r.dropped], ['nit'] * 3)

    def test_process_findings_never_set_the_verdict(self) -> None:
        f = make(path=None, line=None, kind='process', severity='blocker')
        r = self.run_route([(f, keep())])
        self.assertEqual(len(r.process), 1)
        self.assertEqual(len(r.before_merge), 0)
        self.assertEqual(r.verdict, route.VERDICT_CLEAN)
        self.assertEqual(r.reason, '')

    def test_process_low_confidence_and_cap(self) -> None:
        f = make(path=None, line=None, kind='process')
        pairs = [(f, keep('low'))] + [(f, keep())] * (route.MAX_PROCESS + 1)
        r = self.run_route(pairs)
        self.assertEqual(len(r.process), route.MAX_PROCESS)
        self.assertEqual(len(r.dropped), 2)

    def test_reason_is_the_most_severe_posted_title(self) -> None:
        r = self.run_route([(make(title='inline concern'), keep()),
                            (make(path=None, line=None, severity='blocker',
                                  title='pr blocker'), keep())])
        self.assertEqual(r.reason, 'pr blocker')

    def test_verify_only_suggestion_is_not_inline(self) -> None:
        r = self.run_route([(make(suggestion='Please verify this.'),
                             keep())])
        self.assertEqual(r.collapsed[0].note,
                         'suggestion only asks to verify')

    def test_outside_diff_is_collapsed(self) -> None:
        r = self.run_route([(make(line=40), keep())])
        self.assertEqual(r.collapsed[0].note, 'lines are outside the diff')

    def test_one_collapsed_non_blocker_per_file(self) -> None:
        r = self.run_route([(make(line=2), keep('low')),
                            (make(line=3), keep('low')),
                            (make(line=3, severity='blocker'), keep('low'))])
        self.assertEqual(len(r.collapsed), 2)
        self.assertEqual(len(r.dropped), 1)

    def test_inline_cap(self) -> None:
        pairs = [(make(), keep()) for _ in range(route.MAX_INLINE + 2)]
        r = self.run_route(pairs)
        self.assertEqual(len(r.inline), route.MAX_INLINE)

    def test_pr_level_goes_before_merge_never_inline(self) -> None:
        r = self.run_route([(make(path=None, line=None), keep('medium'))])
        self.assertEqual((len(r.inline), len(r.before_merge)), (0, 1))
        self.assertEqual(r.verdict, route.VERDICT_CHANGES)

    def test_pr_level_low_confidence_is_collapsed(self) -> None:
        r = self.run_route([(make(path=None, line=None), keep('low'))])
        self.assertEqual(len(r.before_merge), 0)
        self.assertEqual(len(r.collapsed), 1)

    def test_pr_level_blocker_needs_high_confidence_to_block(self) -> None:
        f = make(path=None, line=None, severity='blocker')
        self.assertEqual(self.run_route([(f, keep('medium'))]).verdict,
                         route.VERDICT_CHANGES)
        self.assertEqual(self.run_route([(f, keep('high'))]).verdict,
                         route.VERDICT_BLOCKING)

    def test_blockers_sort_before_concerns(self) -> None:
        r = self.run_route([(make(title='c'), keep()),
                            (make(title='b', severity='blocker'), keep())])
        self.assertEqual([p.finding.title for p in r.inline], ['b', 'c'])


class TestReplacement(unittest.TestCase):

    def check(self, **over: Any) -> Any:
        return route.check_replacement(make(**over), HEAD)

    def test_valid_replacement(self) -> None:
        self.assertIsNone(self.check(replacement='  x = 1;'))

    def test_rejects_indent_mismatch(self) -> None:
        self.assertIn('indentation', self.check(replacement='x = 1;'))

    def test_rejects_no_change(self) -> None:
        self.assertIn('does not change', self.check(replacement='  x = 2;'))

    def test_rejects_long_replacement(self) -> None:
        self.assertIn('longer', self.check(replacement='\n'.join(
            ['  a;'] * 7)))

    def test_rejects_code_fence(self) -> None:
        self.assertIn('fence', self.check(replacement='  ```'))

    def test_rejects_missing_lines(self) -> None:
        self.assertIn('not found', self.check(line=99, replacement='  a;'))

    def test_failed_replacement_keeps_finding_inline_as_prose(self) -> None:
        r = route.route([(make(replacement='x = 1;'), keep())],
                        diff.parse(DIFF), HEAD)
        self.assertEqual(len(r.inline), 1)
        self.assertIsNone(r.inline[0].replacement)


if __name__ == '__main__':
    unittest.main()
