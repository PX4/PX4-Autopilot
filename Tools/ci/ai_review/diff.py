"""Unified diff parsing: which new-side lines a review comment may target.

GitHub only accepts review comments on lines that appear in the PR diff.
For the new (RIGHT) side that is every added or context line inside a
hunk.
"""

import re
from dataclasses import dataclass, field
from typing import Dict, Set

HUNK_RE = re.compile(r'^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@')


@dataclass
class DiffMap:
    # path -> new-side line numbers that are commentable
    commentable: Dict[str, Set[int]] = field(default_factory=dict)
    # path -> new-side line numbers the PR added or changed
    added: Dict[str, Set[int]] = field(default_factory=dict)
    changed_lines: int = 0

    def can_comment(self, path: str, first: int, last: int) -> bool:
        lines = self.commentable.get(path, set())
        return all(n in lines for n in range(first, last + 1))


def parse(diff_text: str) -> DiffMap:
    result = DiffMap()
    path = None
    in_hunk = False
    new_line = 0
    for raw in diff_text.splitlines():
        if raw.startswith('diff --git '):
            path, in_hunk = None, False
            continue
        if not in_hunk and raw.startswith('+++ '):
            target = raw[4:].strip()
            if target == '/dev/null':
                path = None
            else:
                path = target[2:] if target.startswith('b/') else target
                result.commentable.setdefault(path, set())
                result.added.setdefault(path, set())
            continue
        if not in_hunk and raw.startswith('--- '):
            continue
        match = HUNK_RE.match(raw)
        if match:
            new_line = int(match.group(1))
            in_hunk = True
            continue
        if not in_hunk:
            continue
        if raw.startswith('-'):
            result.changed_lines += 1
            continue
        if path is None:
            continue
        if raw.startswith('+'):
            result.commentable[path].add(new_line)
            result.added[path].add(new_line)
            result.changed_lines += 1
            new_line += 1
        elif raw.startswith(' ') or raw == '':
            result.commentable[path].add(new_line)
            new_line += 1
        # "\ No newline at end of file" and other markers carry no line
    return result
