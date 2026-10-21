"""Finding and verdict data, with strict parsing of model output.

The reviewer and the validator return JSON constrained by the schemas
below (passed to the agent with --json-schema). Parsing re-checks every
field anyway: model output is untrusted. Anything that breaks the
contract is rejected rather than repaired, with one cosmetic exception:
an over-long title or comment is shortened, since losing a finding over
its length helps nobody.

A finding either points at lines of a changed file (path and line set) or
is about the PR as a whole (path and line null). Its kind says whether it
is about the code or about the contribution process (test evidence, the
description, upgrade notes, docs); only code findings decide the verdict.

`comment` is the only finding text posted on the PR. The other text fields
are evidence for the validator and the job-summary report.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

SEVERITIES = ('blocker', 'concern', 'nit')
KINDS = ('code', 'process')
CONFIDENCES = ('high', 'medium', 'low')
CHECKLIST_ITEMS = ('problem', 'tests', 'description', 'compatibility',
                   'docs')
CHECKLIST_STATUSES = ('ok', 'gap', 'not_applicable')

MAX_TITLE = 100
# one sentence; the prompt asks for under 300 characters
MAX_COMMENT = 400
MAX_TEXT = 4000
MAX_FINDINGS = 25

FINDING_FIELDS = {
    'path': {'type': ['string', 'null']},
    'line': {'type': ['integer', 'null'], 'minimum': 1},
    'start_line': {'type': ['integer', 'null'], 'minimum': 1},
    'severity': {'type': 'string', 'enum': list(SEVERITIES)},
    'kind': {'type': 'string', 'enum': list(KINDS)},
    'title': {'type': 'string'},
    'comment': {'type': 'string'},
    'body': {'type': 'string'},
    'trigger': {'type': 'string'},
    'suggestion': {'type': 'string'},
    'replacement': {'type': ['string', 'null']},
    'uncertainty': {'type': 'string'},
    'rule': {'type': ['string', 'null']},
}
REQUIRED_FINDING_FIELDS = (
    'path', 'line', 'severity', 'kind', 'title', 'comment', 'body',
    'trigger', 'suggestion', 'uncertainty')

CHECKLIST_ENTRY = {
    'type': 'object',
    'additionalProperties': False,
    'required': ['status', 'note'],
    'properties': {
        'status': {'type': 'string', 'enum': list(CHECKLIST_STATUSES)},
        'note': {'type': 'string'},
    },
}

REVIEW_SCHEMA: Dict[str, Any] = {
    'type': 'object',
    'additionalProperties': False,
    'required': ['summary', 'checklist', 'findings'],
    'properties': {
        'summary': {'type': 'string'},
        'checklist': {
            'type': 'object',
            'additionalProperties': False,
            'required': list(CHECKLIST_ITEMS),
            'properties': {k: CHECKLIST_ENTRY for k in CHECKLIST_ITEMS},
        },
        'findings': {
            'type': 'array',
            'maxItems': MAX_FINDINGS,
            'items': {
                'type': 'object',
                'additionalProperties': False,
                'required': list(REQUIRED_FINDING_FIELDS),
                'properties': FINDING_FIELDS,
            },
        },
    },
}

VERDICT_SCHEMA: Dict[str, Any] = {
    'type': 'object',
    'additionalProperties': False,
    'required': ['keep', 'confidence', 'reason'],
    'properties': {
        'keep': {'type': 'boolean'},
        'confidence': {'type': 'string', 'enum': list(CONFIDENCES)},
        'reason': {'type': 'string'},
    },
}


class ContractError(ValueError):
    """Model output that does not match the finding contract."""


@dataclass(frozen=True)
class Finding:
    path: Optional[str]
    line: Optional[int]
    start_line: Optional[int]
    severity: str
    kind: str
    title: str
    comment: str
    body: str
    trigger: str
    suggestion: str
    replacement: Optional[str]
    uncertainty: str
    rule: Optional[str]

    @property
    def pr_level(self) -> bool:
        return self.path is None

    @property
    def where(self) -> str:
        return 'PR' if self.path is None else f'{self.path}:{self.line}'

    def to_json(self) -> Dict[str, Any]:
        return dict(self.__dict__)


@dataclass(frozen=True)
class Verdict:
    keep: bool
    confidence: str
    reason: str

    def to_json(self) -> Dict[str, Any]:
        return dict(self.__dict__)


@dataclass(frozen=True)
class ChecklistEntry:
    status: str
    note: str


@dataclass(frozen=True)
class Review:
    summary: str
    findings: List[Finding]
    checklist: Dict[str, ChecklistEntry] = field(default_factory=dict)
    # findings that broke the contract: (raw item, reason), never posted
    rejected: List[Tuple[Any, str]] = field(default_factory=list)


def _text(obj: Dict[str, Any], key: str, limit: int,
          optional: bool = False) -> Optional[str]:
    value = obj.get(key)
    if value is None and optional:
        return None
    if not isinstance(value, str):
        raise ContractError(f'{key}: expected a string')
    if len(value) > limit:
        raise ContractError(f'{key}: longer than {limit} characters')
    return value


def _short_text(obj: Dict[str, Any], key: str, limit: int) -> str:
    value = obj.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ContractError(f'{key}: expected a non-empty string')
    value = ' '.join(value.split())
    if len(value) > limit:
        cut = value[:limit - 1].rsplit(' ', 1)[0]
        value = cut.rstrip(',;:') + '…'
    return value


def _positive_int(obj: Dict[str, Any], key: str,
                  optional: bool = False) -> Optional[int]:
    value = obj.get(key)
    if value is None and optional:
        return None
    # bool is an int subclass; reject it explicitly
    if not isinstance(value, int) or isinstance(value, bool) or value < 1:
        raise ContractError(f'{key}: expected a positive integer')
    return value


def _check_keys(obj: Any, allowed: Any, required: Any, what: str) -> None:
    if not isinstance(obj, dict):
        raise ContractError(f'{what}: expected an object')
    unknown = set(obj) - set(allowed)
    if unknown:
        raise ContractError(f'{what}: unknown fields {sorted(unknown)}')
    missing = set(required) - set(obj)
    if missing:
        raise ContractError(f'{what}: missing fields {sorted(missing)}')


def parse_finding(obj: Any) -> Finding:
    _check_keys(obj, FINDING_FIELDS, REQUIRED_FINDING_FIELDS, 'finding')
    path = _text(obj, 'path', 500, optional=True)
    if path is not None and (
            not path or path.startswith('/') or '..' in path.split('/')):
        raise ContractError('path: must be a relative repository path')
    line = _positive_int(obj, 'line', optional=True)
    start_line = _positive_int(obj, 'start_line', optional=True)
    if (path is None) != (line is None):
        raise ContractError('path and line: set both, or neither for a '
                            'PR-level finding')
    if path is None and start_line is not None:
        raise ContractError('start_line: only for line findings')
    if line is not None and start_line is not None and start_line > line:
        raise ContractError('start_line: must not be after line')
    severity = obj['severity']
    if severity not in SEVERITIES:
        raise ContractError(f'severity: must be one of {SEVERITIES}')
    kind = obj['kind']
    if kind not in KINDS:
        raise ContractError(f'kind: must be one of {KINDS}')
    replacement = _text(obj, 'replacement', MAX_TEXT, optional=True)
    if path is None and replacement is not None:
        raise ContractError('replacement: only for line findings')
    return Finding(
        path=path,
        line=line,
        start_line=start_line,
        severity=severity,
        kind=kind,
        title=_short_text(obj, 'title', MAX_TITLE),
        comment=_short_text(obj, 'comment', MAX_COMMENT),
        body=_text(obj, 'body', MAX_TEXT) or '',
        trigger=_text(obj, 'trigger', MAX_TEXT) or '',
        suggestion=_text(obj, 'suggestion', MAX_TEXT) or '',
        replacement=replacement,
        uncertainty=_text(obj, 'uncertainty', MAX_TEXT) or '',
        rule=_text(obj, 'rule', 500, optional=True),
    )


def parse_checklist(obj: Any) -> Dict[str, ChecklistEntry]:
    _check_keys(obj, CHECKLIST_ITEMS, CHECKLIST_ITEMS, 'checklist')
    out = {}
    for key in CHECKLIST_ITEMS:
        entry = obj[key]
        _check_keys(entry, ('status', 'note'), ('status', 'note'),
                    f'checklist.{key}')
        if entry['status'] not in CHECKLIST_STATUSES:
            raise ContractError(
                f'checklist.{key}.status: must be one of {CHECKLIST_STATUSES}')
        out[key] = ChecklistEntry(status=entry['status'],
                                  note=_text(entry, 'note', MAX_TEXT) or '')
    return out


def parse_review(obj: Any) -> Review:
    keys = ('summary', 'checklist', 'findings')
    _check_keys(obj, keys, keys, 'review')
    summary = _text(obj, 'summary', MAX_TEXT) or ''
    checklist = parse_checklist(obj['checklist'])
    raw = obj['findings']
    if not isinstance(raw, list):
        raise ContractError('findings: expected an array')
    if len(raw) > MAX_FINDINGS:
        raise ContractError(f'findings: more than {MAX_FINDINGS}')
    # One malformed finding must not discard an otherwise valid review;
    # it is rejected on its own and reported, never posted.
    good: List[Finding] = []
    rejected: List[Tuple[Any, str]] = []
    for item in raw:
        try:
            good.append(parse_finding(item))
        except ContractError as e:
            rejected.append((item, str(e)))
    return Review(summary=summary, findings=good, checklist=checklist,
                  rejected=rejected)


def parse_verdict(obj: Any) -> Verdict:
    _check_keys(obj, ('keep', 'confidence', 'reason'),
                ('keep', 'confidence', 'reason'), 'verdict')
    if not isinstance(obj['keep'], bool):
        raise ContractError('keep: expected a boolean')
    if obj['confidence'] not in CONFIDENCES:
        raise ContractError(f'confidence: must be one of {CONFIDENCES}')
    return Verdict(keep=obj['keep'], confidence=obj['confidence'],
                   reason=_text(obj, 'reason', MAX_TEXT) or '')
