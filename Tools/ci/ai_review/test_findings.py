"""Model output is untrusted: malformed findings must be rejected, not
repaired, so nothing outside the contract can reach a PR comment."""

import unittest
from typing import Any, Dict

from ai_review import findings


def finding(**over: Any) -> Dict[str, Any]:
    base: Dict[str, Any] = {
        'path': 'src/modules/foo/Foo.cpp', 'line': 12, 'start_line': None,
        'severity': 'concern', 'kind': 'code', 'title': 't',
        'comment': 'Clamp dt before integrating.', 'body': 'b', 'trigger': 'x',
        'suggestion': 'Clamp dt to [1 ms, 100 ms] before integrating.',
        'replacement': None, 'uncertainty': '', 'rule': None}
    base.update(over)
    return base


class TestParseFinding(unittest.TestCase):

    def test_accepts_valid(self) -> None:
        f = findings.parse_finding(finding(start_line=10))
        self.assertEqual((f.path, f.start_line, f.line), (
            'src/modules/foo/Foo.cpp', 10, 12))

    def test_rejects_unknown_field(self) -> None:
        with self.assertRaises(findings.ContractError):
            findings.parse_finding(finding(approve=True))

    def test_rejects_missing_required(self) -> None:
        data = finding()
        del data['trigger']
        with self.assertRaises(findings.ContractError):
            findings.parse_finding(data)

    def test_rejects_paths_outside_repo(self) -> None:
        for path in ('/etc/passwd', '../x.cpp', 'a/../../x', ''):
            with self.subTest(path=path):
                with self.assertRaises(findings.ContractError):
                    findings.parse_finding(finding(path=path))

    def test_rejects_bad_lines(self) -> None:
        for over in ({'line': 0}, {'line': True}, {'line': '3'},
                     {'start_line': 20}):
            with self.subTest(over=over):
                with self.assertRaises(findings.ContractError):
                    findings.parse_finding(finding(**over))

    def test_rejects_unknown_severity(self) -> None:
        with self.assertRaises(findings.ContractError):
            findings.parse_finding(finding(severity='critical'))

    def test_rejects_unknown_kind(self) -> None:
        with self.assertRaises(findings.ContractError):
            findings.parse_finding(finding(kind='style'))

    def test_rejects_empty_comment(self) -> None:
        with self.assertRaises(findings.ContractError):
            findings.parse_finding(finding(comment=' '))

    def test_rejects_oversized_text(self) -> None:
        with self.assertRaises(findings.ContractError):
            findings.parse_finding(finding(body='x' * 5000))

    def test_long_title_is_shortened_not_rejected(self) -> None:
        # a real Opus finding was lost to this once
        f = findings.parse_finding(finding(title='word ' * 40))
        self.assertLessEqual(len(f.title), findings.MAX_TITLE)
        self.assertTrue(f.title.endswith('…'))

    def test_long_comment_is_shortened_not_rejected(self) -> None:
        f = findings.parse_finding(finding(comment='word ' * 200))
        self.assertLessEqual(len(f.comment), findings.MAX_COMMENT)
        self.assertTrue(f.comment.endswith('…'))

    def test_pr_level_finding(self) -> None:
        f = findings.parse_finding(finding(path=None, line=None))
        self.assertTrue(f.pr_level)
        self.assertEqual(f.where, 'PR')

    def test_pr_level_rejects_half_anchors_and_replacements(self) -> None:
        for over in ({'path': None}, {'line': None},
                     {'path': None, 'line': None, 'start_line': 3},
                     {'path': None, 'line': None, 'replacement': 'x'}):
            with self.subTest(over=over):
                with self.assertRaises(findings.ContractError):
                    findings.parse_finding(finding(**over))


def checklist(**over: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {k: {'status': 'ok', 'note': ''}
                           for k in findings.CHECKLIST_ITEMS}
    for k, status in over.items():
        out[k] = {'status': status, 'note': 'n'}
    return out


class TestParseReviewAndVerdict(unittest.TestCase):

    def test_review_caps_finding_count(self) -> None:
        raw = {'summary': 's', 'checklist': checklist(),
               'findings': [finding()] * 26}
        with self.assertRaises(findings.ContractError):
            findings.parse_review(raw)

    def test_bad_finding_is_rejected_alone(self) -> None:
        raw = {'summary': 's', 'checklist': checklist(),
               'findings': [finding(), finding(path='/etc/passwd')]}
        review = findings.parse_review(raw)
        self.assertEqual(len(review.findings), 1)
        self.assertEqual(len(review.rejected), 1)
        self.assertIn('path', review.rejected[0][1])

    def test_checklist_requires_every_item(self) -> None:
        partial = checklist(tests='gap')
        del partial['docs']
        with self.assertRaises(findings.ContractError):
            findings.parse_review({'summary': 's', 'checklist': partial,
                                   'findings': []})

    def test_checklist_parsed(self) -> None:
        review = findings.parse_review({
            'summary': 's', 'checklist': checklist(tests='gap'),
            'findings': []})
        self.assertEqual(review.checklist['tests'].status, 'gap')

    def test_verdict_requires_bool_keep(self) -> None:
        with self.assertRaises(findings.ContractError):
            findings.parse_verdict({'keep': 'yes', 'confidence': 'high',
                                    'reason': 'r'})

    def test_verdict_valid(self) -> None:
        v = findings.parse_verdict({'keep': True, 'confidence': 'medium',
                                    'reason': 'r'})
        self.assertEqual(v.confidence, 'medium')

    def test_schemas_list_the_same_fields_the_parser_accepts(self) -> None:
        item = findings.REVIEW_SCHEMA['properties']['findings']['items']
        self.assertEqual(set(item['properties']),
                         set(findings.FINDING_FIELDS))
        self.assertEqual(set(item['required']),
                         set(findings.REQUIRED_FINDING_FIELDS))


if __name__ == '__main__':
    unittest.main()
