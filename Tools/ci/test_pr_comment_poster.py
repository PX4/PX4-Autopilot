"""The sticky-comment poster accepts a delete manifest and only removes that comment."""

from contextlib import redirect_stderr, redirect_stdout
from importlib.util import module_from_spec, spec_from_file_location
from io import StringIO
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
_SPEC = spec_from_file_location(
    "pr_comment_poster", Path(__file__).with_name("pr-comment-poster.py"))
_POSTER = module_from_spec(_SPEC)
_SPEC.loader.exec_module(_POSTER)

FLASH_MARKER = "<!-- pr-comment-poster:flash-analysis -->"


class FakeClient:
    def __init__(self, comments):
        self.comments = comments
        self.requests = []

    def paginated(self, path):
        return self.comments

    def request(self, method, path, json_body=None):
        self.requests.append((method, path, json_body))
        return None, {}


class PrCommentPosterTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self._n = 0

    def manifest(self, mode, marker=FLASH_MARKER, body=None):
        self._n += 1
        artifact = self.root / f"{mode}-{self._n}"
        artifact.mkdir()
        (artifact / "manifest.json").write_text(json.dumps({
            "pr_number": 28789,
            "marker": marker,
            "mode": mode,
        }))
        if body is not None:
            (artifact / "body.md").write_text(body)
        return artifact

    def assert_rejected(self, artifact, text):
        buffer = StringIO()
        with redirect_stderr(buffer), self.assertRaises(SystemExit) as caught:
            _POSTER.validate_manifest(str(artifact))
        self.assertEqual(caught.exception.code, 1)
        self.assertIn(text, buffer.getvalue())

    def test_delete_manifest_does_not_require_a_body(self):
        result = _POSTER.validate_manifest(str(self.manifest("delete")))
        self.assertEqual(result["mode"], "delete")
        self.assertEqual(result["body"], "")
        self.assertEqual(result["pr_number"], 28789)

    def test_delete_marker_must_be_one_producer_tag(self):
        self.assert_rejected(self.manifest("delete", marker="<!-- pr-comment-poster:"), "tag")
        self.assert_rejected(
            self.manifest("delete", marker=FLASH_MARKER + " trailing"),
            "tag",
        )
        docs = _POSTER.validate_manifest(str(self.manifest(
            "delete", marker="<!-- pr-comment-poster:docs-link-check -->",
        )))
        self.assertEqual(docs["marker"], "<!-- pr-comment-poster:docs-link-check -->")

    def test_upsert_still_requires_a_body(self):
        self.assert_rejected(self.manifest("upsert"), "body.md missing")
        result = _POSTER.validate_manifest(str(self.manifest("upsert", body="hello\n")))
        self.assertEqual(result["body"], "hello\n")

    def test_delete_removes_only_the_matching_comment(self):
        client = FakeClient([
            {"id": 1, "body": "unrelated"},
            {"id": 9, "body": FLASH_MARKER + "\n## zero\n"},
        ])
        with redirect_stdout(StringIO()):
            _POSTER.delete_comment(client, "PX4/PX4-Autopilot", 28789, FLASH_MARKER)
        self.assertEqual(client.requests, [(
            "DELETE",
            "repos/PX4/PX4-Autopilot/issues/comments/9",
            None,
        )])

    def test_delete_is_a_no_op_when_the_comment_is_gone(self):
        client = FakeClient([])
        with redirect_stdout(StringIO()):
            _POSTER.delete_comment(client, "PX4/PX4-Autopilot", 28789, FLASH_MARKER)
        self.assertEqual(client.requests, [])


if __name__ == "__main__":
    unittest.main()
