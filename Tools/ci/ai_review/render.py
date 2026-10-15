"""Turn a routed review into the pr-review artifact and a readable report.

The artifact follows the contract in .github/workflows/pr-review-poster.yml
(manifest.json + comments.json). Rendering never adds model text outside
the comment bodies and the summary field, both of which the poster sends
to the GitHub API as JSON fields.
"""

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from .findings import CHECKLIST_ITEMS, ChecklistEntry
from .route import Placed, Routed

CHECKLIST_LABELS = {
    'problem': 'problem is real',
    'tests': 'test evidence',
    'description': 'description matches the change',
    'compatibility': 'compatibility',
    'docs': 'docs and changelog',
}
MARKER = '<!-- pr-review-poster:ai-review -->'
FINDING_TAG = '<!-- ai-review-finding -->'
SUMMARY_LIMIT = 60000
README_URL = ('https://github.com/PX4/PX4-Autopilot/blob/main/'
              'Tools/ci/ai_review/README.md')


def _label(severity: str) -> str:
    return 'Blocker' if severity == 'blocker' else 'Must-fix'


def comment_body(p: Placed) -> str:
    f = p.finding
    parts = [f'**{_label(f.severity)}:** {f.comment}']
    if p.replacement is not None:
        parts += ['', '```suggestion', p.replacement, '```']
    parts += ['', FINDING_TAG]
    return '\n'.join(parts)


def comments(routed: Routed) -> List[Dict[str, Any]]:
    out = []
    for p in routed.inline:
        f = p.finding
        entry: Dict[str, Any] = {'path': f.path, 'line': f.line,
                                 'side': 'RIGHT', 'body': comment_body(p)}
        if f.start_line is not None and f.start_line != f.line:
            entry['start_line'] = f.start_line
            entry['start_side'] = 'RIGHT'
        out.append(entry)
    return out


def _item(p: Placed, note: bool = False) -> str:
    f = p.finding
    where = '' if f.pr_level else f'`{f.where}`: '
    blocker = '**Blocker:** ' if f.severity == 'blocker' else ''
    why = f' ({p.note})' if note and p.note else ''
    return f'- {where}{blocker}{f.comment}{why}'


def cost_line(usage: Optional[Dict[str, Any]], model_name: str) -> str:
    """'Cost: $1.23 · Claude Opus 5.5 · 4 model calls · 7 min'.

    Claude Code's own estimate at list prices; on Bedrock it matched a
    recomputation from token counts to the cent.
    """
    if not usage or 'cost_usd' not in usage:
        return ''
    parts = [f'Cost: ${usage["cost_usd"]:.2f}', model_name,
             f'{usage.get("calls", 0)} model call(s)']
    seconds = usage.get('wall_seconds')
    if isinstance(seconds, (int, float)):
        parts.append(f'{max(1, round(seconds / 60))} min')
    return ' · '.join(parts)


def summary(routed: Routed, model_name: str,
            usage: Optional[Dict[str, Any]] = None,
            report_url: str = '') -> str:
    """The review body: verdict, then one line per posted finding.

    The model's summary, the checklist, nits and each finding's evidence
    are left to the job-summary report, linked from the footer.
    """
    head = f'**AI review: {routed.verdict}.**'
    if routed.reason:
        head += f' {routed.reason}'
    parts = [head]
    for title, items, note in (('Must-fix', routed.before_merge, False),
                               ('Worth checking', routed.collapsed, True),
                               ('Process', routed.process, False)):
        if items:
            parts += ['', f'**{title}**']
            parts += [_item(p, note) for p in items]
    links = [f'[How this review works]({README_URL})']
    if report_url:
        links.append(f'[Full report]({report_url})')
    cost = cost_line(usage, model_name)
    parts += ['', f'<sub>{model_name}, checked by a second, independent '
              f'pass. It can be wrong; maintainers decide. '
              + ' · '.join(links) + (f'<br>{cost}' if cost else '')
              + '</sub>']
    text = '\n'.join(parts)
    if len(text.encode('utf-8')) > SUMMARY_LIMIT:
        text = text.encode('utf-8')[:SUMMARY_LIMIT - 200].decode(
            'utf-8', 'ignore') + '\n\n(truncated)'
    return text


def find_leaks(texts: Iterable[str], secrets: Iterable[str]) -> List[str]:
    """Return the secrets (by position) that appear in any output text.

    The agent runs with AWS session credentials in its environment. Even
    though they only allow invoking two Bedrock models, nothing derived
    from them may reach a public PR comment.
    """
    values = [s for s in secrets if len(s) >= 12]
    blob = '\n'.join(texts)
    return [f'secret #{i}' for i, s in enumerate(values) if s in blob]


def write_artifact(out_dir: Path, pr_number: int, commit_sha: str,
                   routed: Routed, model_name: str,
                   usage: Optional[Dict[str, Any]] = None,
                   report_url: str = '') -> Dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        'pr_number': pr_number,
        'marker': MARKER,
        'event': 'COMMENT',
        'commit_sha': commit_sha,
        'summary': summary(routed, model_name, usage, report_url),
        # a review with only a summary is still a review; and each run
        # replaces the previous one instead of piling up
        'post_without_comments': True,
        'supersede_previous': True,
    }
    body = comments(routed)
    (out_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    (out_dir / 'comments.json').write_text(json.dumps(body, indent=2))
    return {'manifest': manifest, 'comments': body}


def refusal_summary(e: Any, model_name: str,
                    usage: Optional[Dict[str, Any]] = None) -> str:
    """Review body when the model declined: not a review, with the reason.

    The reason is the model provider's own text, passed through verbatim
    in a code block so it cannot be mistaken for findings.
    """
    lines = [f'**Automated review not done.** {model_name} declined to '
             f'review this pull request, so no findings were produced and '
             f'nothing in this PR was checked by the automated reviewer.',
             '', f'Reason category: `{e.category}`'
             + (f' · Request ID: `{e.request_id}`' if e.request_id else ''),
             '', 'Message from the model provider:', '', '```text',
             e.reason.replace('```', "'''"), '```', '',
             'This is usually a false positive from the provider\'s safety '
             'classifier, not a judgement about the PR. A maintainer can '
             're-run the review or review it by hand.', '',
             f'<sub>[How this review works]({README_URL})'
             + (f'<br>{cost_line(usage, model_name)}'
                if cost_line(usage, model_name) else '') + '</sub>']
    return '\n'.join(lines)


def write_refusal(out_dir: Path, pr_number: int, commit_sha: str, e: Any,
                  model_name: str,
                  usage: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        'pr_number': pr_number,
        'marker': MARKER,
        'event': 'COMMENT',
        'commit_sha': commit_sha,
        'summary': refusal_summary(e, model_name, usage),
        # the notice must reach the PR even though there are no findings
        'post_without_comments': True,
        'supersede_previous': True,
    }
    (out_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    (out_dir / 'comments.json').write_text('[]')
    return {'manifest': manifest, 'comments': []}


def refusal_report(e: Any, artifact: Dict[str, Any],
                   usage: Dict[str, Any]) -> str:
    return '\n'.join([
        '## AI review (pilot report)', '',
        f'**Not reviewed: the model declined** (`{e.category}`).', '',
        f'Cost before the refusal: ${usage.get("cost_usd", 0):.2f}.', '',
        '### Review body', '', artifact['manifest']['summary'], '']) + '\n'


def _details(placement: str, p: Placed) -> List[str]:
    f = p.finding
    lines = [f'#### {placement}: {f.severity}, {f.kind}, `{f.where}`: '
             f'{f.title}', '',
             f'- **Posted text:** {f.comment}',
             f'- **Evidence:** {f.body}',
             f'- **When it happens:** {f.trigger}',
             f'- **Suggested fix:** {f.suggestion}']
    if f.uncertainty.strip():
        lines.append(f'- **Not certain:** {f.uncertainty}')
    if f.rule:
        lines.append(f'- **Rule:** {f.rule}')
    lines.append(f'- **Validator** ({p.verdict.confidence} confidence): '
                 f'{p.verdict.reason}')
    if p.note:
        lines.append(f'- **Why {placement.lower()}:** {p.note}')
    return lines + ['']


def report_markdown(routed: Routed, artifact: Dict[str, Any],
                    usage: Dict[str, Any], model_summary: str = '',
                    checklist: Optional[Dict[str, ChecklistEntry]] = None
                    ) -> str:
    """Job-summary report: what was posted, then every finding in full."""
    lines = ['## AI review (pilot report)', '',
             f'Verdict: **{routed.verdict}**. Inline: {len(routed.inline)},'
             f' must-fix: {len(routed.before_merge)}, worth checking: '
             f'{len(routed.collapsed)}, process: {len(routed.process)}, '
             f'not posted: {len(routed.dropped)}.',
             '',
             f'Cost estimate: ${usage.get("cost_usd", 0):.2f} over '
             f'{usage.get("calls", 0)} model call(s). Findings rejected '
             f'for breaking the output contract: '
             f'{usage.get("rejected_findings", 0)}. Sandbox: '
             f'{usage.get("sandbox_calls", 0)} call(s), '
             f'{usage.get("sandbox_failed_calls", 0)} failed, '
             f'{usage.get("sandbox_seconds", 0)} s.', '',
             '### Review body', '', artifact['manifest']['summary'], '',
             '### Inline comments', '']
    for c in artifact['comments']:
        lines += [f'#### `{c["path"]}:{c["line"]}`', '', c['body'], '']
    if model_summary.strip():
        lines += ['### Model summary', '', model_summary.strip(), '']
    if checklist:
        lines += ['### Checklist', '']
        for k in CHECKLIST_ITEMS:
            if k in checklist:
                e = checklist[k]
                lines.append(f'- **{CHECKLIST_LABELS[k]}** ({e.status}): '
                             f'{e.note}')
        lines.append('')
    lines += ['### All findings', '']
    for placement, items in (('Inline', routed.inline),
                             ('Must-fix', routed.before_merge),
                             ('Worth checking', routed.collapsed),
                             ('Process', routed.process),
                             ('Not posted', routed.dropped)):
        for p in items:
            lines += _details(placement, p)
    return '\n'.join(lines) + '\n'
