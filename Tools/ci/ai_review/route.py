"""Decide where each finding goes, from validated findings to a review.

The model proposes; this module decides:

- Nits are never posted; they stay in the job-summary report.
- Process findings (test evidence, description, upgrade notes, docs) go
  to the "Process" list of the review body when the validator kept them
  with high or medium confidence. They never change the verdict.
- Line findings reach the PR inline only when the independent validator
  kept them with high confidence, their suggestion is more than "please
  verify", and their lines are in the diff.
- Other PR-level code findings go to the "Must-fix" list of the review
  body when the validator kept them with high or medium confidence.
- Code findings that miss these bars go to "Worth checking", one per file.

The overall verdict is derived from the posted code findings, never taken
from the model.
"""

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .diff import DiffMap
from .findings import Finding, Verdict

MAX_INLINE = 10
MAX_BEFORE_MERGE = 8
MAX_PROCESS = 4
MAX_REPLACEMENT_LINES = 6
SEVERITY_ORDER = {'blocker': 0, 'concern': 1, 'nit': 2}
CONFIDENCE_ORDER = {'high': 0, 'medium': 1, 'low': 2}
# "Please verify X" is not a suggestion; it hands the work back
VERIFY_ONLY_RE = re.compile(
    r'^\s*(please\s+)?(verify|ensure|make sure|confirm|double[- ]check|'
    r'check (that|whether|if))\b', re.IGNORECASE)

VERDICT_BLOCKING = "don't merge"
VERDICT_CHANGES = 'merge after fixes'
VERDICT_CLEAN = 'no code issues found'


@dataclass
class Placed:
    finding: Finding
    verdict: Verdict
    replacement: Optional[str]
    note: str = ''


@dataclass
class Routed:
    inline: List[Placed] = field(default_factory=list)
    before_merge: List[Placed] = field(default_factory=list)
    process: List[Placed] = field(default_factory=list)
    collapsed: List[Placed] = field(default_factory=list)
    # not posted; `note` says why
    dropped: List[Placed] = field(default_factory=list)

    @property
    def verdict(self) -> str:
        posted = self.inline + self.before_merge
        if any(p.finding.severity == 'blocker'
               and p.verdict.confidence == 'high' for p in posted):
            return VERDICT_BLOCKING
        if posted:
            return VERDICT_CHANGES
        return VERDICT_CLEAN

    @property
    def reason(self) -> str:
        """Title of the posted code finding that decides the verdict."""
        posted = self.inline + self.before_merge
        if not posted:
            return ''
        return min(posted, key=lambda p: SEVERITY_ORDER[p.finding.severity]
                   ).finding.title


def _span(f: Finding) -> Tuple[int, int]:
    assert f.line is not None
    return (f.start_line or f.line, f.line)


def check_replacement(f: Finding,
                      head_files: Dict[str, List[str]]) -> Optional[str]:
    """Return why the replacement cannot be a suggestion block, or None.

    A suggestion block replaces exactly the anchored lines, so it is only
    offered when that is a complete, minimal and well-formed fix.
    """
    if f.replacement is None or f.path is None:
        return 'no replacement'
    first, last = _span(f)
    if last - first + 1 > MAX_REPLACEMENT_LINES:
        return f'anchors more than {MAX_REPLACEMENT_LINES} lines'
    new_lines = f.replacement.split('\n')
    if len(new_lines) > MAX_REPLACEMENT_LINES:
        return f'replacement longer than {MAX_REPLACEMENT_LINES} lines'
    if '```' in f.replacement:
        return 'replacement contains a code fence'
    source = head_files.get(f.path)
    if source is None or last > len(source):
        return 'anchored lines not found in the PR head'
    old_lines = source[first - 1:last]
    if [s.rstrip() for s in old_lines] == [s.rstrip() for s in new_lines]:
        return 'replacement does not change anything'

    def indent(s: str) -> str:
        return s[:len(s) - len(s.lstrip())]

    old_first = next((s for s in old_lines if s.strip()), '')
    new_first = next((s for s in new_lines if s.strip()), '')
    if indent(old_first) != indent(new_first):
        return 'replacement indentation does not match the code'
    return None


def _line_reason(p: Placed, diff: DiffMap, inline_count: int) -> Optional[str]:
    f = p.finding
    assert f.path is not None
    if p.verdict.confidence != 'high':
        return f'{p.verdict.confidence} confidence'
    if VERIFY_ONLY_RE.match(f.suggestion):
        return 'suggestion only asks to verify'
    if not diff.can_comment(f.path, *_span(f)):
        return 'lines are outside the diff'
    if inline_count >= MAX_INLINE:
        return f'more than {MAX_INLINE} inline findings'
    return None


def _listed_reason(p: Placed, listed: int, cap: int,
                   what: str) -> Optional[str]:
    if p.verdict.confidence == 'low':
        return 'low confidence'
    if VERIFY_ONLY_RE.match(p.finding.suggestion):
        return 'suggestion only asks to verify'
    if listed >= cap:
        return f'more than {cap} {what} findings'
    return None


def route(pairs: List[Tuple[Finding, Verdict]], diff: DiffMap,
          head_files: Dict[str, List[str]]) -> Routed:
    routed = Routed()
    candidates: List[Placed] = []
    for f, v in pairs:
        p = Placed(finding=f, verdict=v, replacement=None)
        if not v.keep:
            p.note = f'validator: {v.reason}'
            routed.dropped.append(p)
            continue
        candidates.append(p)

    candidates.sort(key=lambda p: (SEVERITY_ORDER[p.finding.severity],
                                   CONFIDENCE_ORDER[p.verdict.confidence],
                                   p.finding.where))
    collapsed_files: Dict[str, int] = {}
    for p in candidates:
        f = p.finding
        if f.severity == 'nit':
            p.note = 'nit'
            routed.dropped.append(p)
            continue
        if f.kind == 'process':
            reason = _listed_reason(p, len(routed.process), MAX_PROCESS,
                                    'process')
            if reason is None:
                routed.process.append(p)
            else:
                p.note = reason
                routed.dropped.append(p)
            continue
        if f.pr_level:
            reason = _listed_reason(p, len(routed.before_merge),
                                    MAX_BEFORE_MERGE, 'PR-level')
            if reason is None:
                routed.before_merge.append(p)
                continue
        else:
            reason = _line_reason(p, diff, len(routed.inline))
            if reason is None:
                problem = check_replacement(f, head_files)
                p.replacement = f.replacement if problem is None else None
                routed.inline.append(p)
                continue

        p.note = reason
        key = f.path or 'PR'
        # one non-blocker per file keeps the collapsed list readable
        if f.severity != 'blocker' and collapsed_files.get(key, 0) >= 1:
            p.note = f'{reason}; already listed'
            routed.dropped.append(p)
            continue
        if f.severity != 'blocker':
            collapsed_files[key] = collapsed_files.get(key, 0) + 1
        routed.collapsed.append(p)
    return routed


def load_head_files(checkout: Path, paths: List[str]) -> Dict[str, List[str]]:
    files: Dict[str, List[str]] = {}
    root = checkout.resolve()
    for rel in paths:
        target = (root / rel).resolve()
        if root not in target.parents or not target.is_file():
            continue
        try:
            files[rel] = target.read_text(encoding='utf-8').split('\n')
        except (OSError, UnicodeDecodeError):
            continue
    return files
