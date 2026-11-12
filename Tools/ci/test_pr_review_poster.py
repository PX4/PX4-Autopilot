#!/usr/bin/env python3
"""Tests for pr-review-poster.py's opt-in behaviours.

The poster is shared by clang-tidy and the AI review. These tests pin that
a manifest without the new fields behaves exactly as before (no review for
an empty comment list, COMMENT reviews left alone), and that the opt-in
fields post a summary-only review and supersede earlier COMMENT reviews
without touching anyone else's review.
"""

import importlib.util
import json
import os
import tempfile
import unittest
from typing import Any, Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    'pr_review_poster', os.path.join(HERE, 'pr-review-poster.py'))
assert _spec and _spec.loader
poster: Any = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(poster)

MARKER = '<!-- pr-review-poster:ai-review -->'
SHA = 'a' * 40


class FakeClient:

    def __init__(self, reviews: Optional[List[Dict[str, Any]]] = None
                 ) -> None:
        self.reviews = reviews or []
        self.requests: List[Tuple[str, str, Any]] = []

    def paginated(self, path: str) -> List[Dict[str, Any]]:
        return self.reviews

    def request(self, method: str, path: str,
                json_body: Any = None) -> Tuple[Any, Dict[str, str]]:
        self.requests.append((method, path, json_body))
        return {}, {}


def write_artifact(tmp: str, manifest: Dict[str, Any],
                   comments: List[Dict[str, Any]]) -> None:
    with open(os.path.join(tmp, 'manifest.json'), 'w') as f:
        json.dump(manifest, f)
    with open(os.path.join(tmp, 'comments.json'), 'w') as f:
        json.dump(comments, f)


def manifest(**extra: Any) -> Dict[str, Any]:
    base = {'pr_number': 7, 'marker': MARKER, 'event': 'COMMENT',
            'commit_sha': SHA, 'summary': 'Summary.'}
    base.update(extra)
    return base


class TestManifestFlags(unittest.TestCase):

    def validate(self, m: Dict[str, Any]) -> Dict[str, Any]:
        with tempfile.TemporaryDirectory() as tmp:
            write_artifact(tmp, m, [])
            result: Dict[str, Any] = poster.validate_manifest(tmp)
            return result

    def test_flags_default_to_false(self) -> None:
        # clang-tidy's manifest carries neither field
        result = self.validate(manifest())
        self.assertFalse(result['post_without_comments'])
        self.assertFalse(result['supersede_previous'])

    def test_flags_must_be_booleans(self) -> None:
        for key in ('post_without_comments', 'supersede_previous'):
            with self.subTest(key=key):
                with self.assertRaises(SystemExit):
                    self.validate(manifest(**{key: 'yes'}))

    def test_post_without_comments_needs_a_summary(self) -> None:
        with self.assertRaises(SystemExit):
            self.validate(manifest(summary='  ', post_without_comments=True))


class TestPostReview(unittest.TestCase):

    def post(self, comments: List[Dict[str, Any]], allow_empty: bool
             ) -> FakeClient:
        client = FakeClient()
        poster.post_review(client, 'PX4/PX4-Autopilot', 7, MARKER,
                           'COMMENT', SHA, 'Summary.', comments,
                           allow_empty=allow_empty)
        return client

    def test_empty_without_opt_in_posts_nothing(self) -> None:
        self.assertEqual(self.post([], allow_empty=False).requests, [])

    def test_empty_with_opt_in_posts_summary_only_comment_review(
            self) -> None:
        reqs = self.post([], allow_empty=True).requests
        self.assertEqual(len(reqs), 1)
        method, path, body = reqs[0]
        self.assertEqual((method, path),
                         ('POST', 'repos/PX4/PX4-Autopilot/pulls/7/reviews'))
        self.assertEqual(body['event'], 'COMMENT')
        self.assertEqual(body['comments'], [])
        self.assertIn('Summary.', body['body'])
        self.assertTrue(body['body'].startswith(MARKER))

    def test_comments_post_as_before(self) -> None:
        c = {'path': 'a.cpp', 'line': 3, 'side': 'RIGHT', 'body': 'x'}
        reqs = self.post([c], allow_empty=True).requests
        self.assertEqual(len(reqs), 1)
        self.assertEqual(len(reqs[0][2]['comments']), 1)


def review(rid: int, login: str, body: str,
           state: str = 'COMMENTED') -> Dict[str, Any]:
    return {'id': rid, 'user': {'login': login}, 'body': body,
            'state': state}


class TestSupersede(unittest.TestCase):

    def test_only_our_comment_reviews_are_edited(self) -> None:
        client = FakeClient([
            review(1, poster.BOT_LOGIN, MARKER + '\nold findings'),
            review(2, 'alice', MARKER + '\nhuman quoting the marker'),
            review(3, poster.BOT_LOGIN, '<!-- other-tool -->\nnot ours'),
            review(4, poster.BOT_LOGIN, MARKER + '\nx', state='DISMISSED'),
            review(5, poster.BOT_LOGIN,
                   MARKER + '\n' + poster.SUPERSEDED_TAG + '\nalready'),
        ])
        edited = poster.supersede_comment_reviews(
            client, 'PX4/PX4-Autopilot', 7, MARKER)
        self.assertEqual(edited, 1)
        method, path, body = client.requests[0]
        self.assertEqual((method, path),
                         ('PUT', 'repos/PX4/PX4-Autopilot/pulls/7/reviews/1'))
        self.assertIn(poster.SUPERSEDED_TAG, body['body'])
        self.assertIn(MARKER, body['body'])
        self.assertIn('old findings', body['body'])

    def test_dismissal_still_skips_comment_reviews(self) -> None:
        # unchanged behaviour for producers that do not opt in
        client = FakeClient([review(1, poster.BOT_LOGIN, MARKER)])
        poster.dismiss_stale_reviews(client, 'PX4/PX4-Autopilot', 7, MARKER)
        self.assertEqual(client.requests, [])


if __name__ == '__main__':
    unittest.main()
