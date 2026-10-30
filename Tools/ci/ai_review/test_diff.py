"""GitHub rejects review comments on lines outside the diff, so the
commentable-line map must match GitHub's view of the new side."""

import unittest

from ai_review import diff

SAMPLE = """\
diff --git a/src/a.cpp b/src/a.cpp
index 111..222 100644
--- a/src/a.cpp
+++ b/src/a.cpp
@@ -10,4 +10,5 @@ void f()
 int a = 1;
-int b = 2;
+int b = 3;
+int c = 4;
 return a;
\\ No newline at end of file
diff --git a/old.txt b/old.txt
deleted file mode 100644
--- a/old.txt
+++ /dev/null
@@ -1,2 +0,0 @@
-gone
-too
diff --git a/new.py b/new.py
new file mode 100644
--- /dev/null
+++ b/new.py
@@ -0,0 +1,2 @@
+x = 1
+y = 2
"""


class TestParse(unittest.TestCase):

    def setUp(self) -> None:
        self.d = diff.parse(SAMPLE)

    def test_context_and_added_lines_are_commentable(self) -> None:
        self.assertEqual(self.d.commentable['src/a.cpp'], {10, 11, 12, 13})
        self.assertEqual(self.d.added['src/a.cpp'], {11, 12})

    def test_new_file(self) -> None:
        self.assertEqual(self.d.commentable['new.py'], {1, 2})

    def test_deleted_file_has_no_new_side(self) -> None:
        self.assertNotIn('old.txt', self.d.commentable)

    def test_changed_line_count(self) -> None:
        # a.cpp: 1 removed + 2 added; old.txt: 2 removed; new.py: 2 added
        self.assertEqual(self.d.changed_lines, 7)

    def test_can_comment_requires_every_line(self) -> None:
        self.assertTrue(self.d.can_comment('src/a.cpp', 11, 12))
        self.assertFalse(self.d.can_comment('src/a.cpp', 13, 14))
        self.assertFalse(self.d.can_comment('missing.cpp', 1, 1))


if __name__ == '__main__':
    unittest.main()
