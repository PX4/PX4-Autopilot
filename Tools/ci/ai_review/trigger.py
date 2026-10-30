"""Decide whether a workflow event may start a review, and with what.

Two ways in:

- workflow_dispatch: anyone who can dispatch already has write access;
  the PR number, model and effort come from the inputs.
- issue_comment: a `!ai-review` comment on a pull request. The commenter's
  repository permission is looked up through the API and must be write,
  maintain or admin. author_association is not used: an organization
  member can have read-only access to this repository.

Writes the decision to $GITHUB_OUTPUT as run, pr, head_sha, model,
effort, comment_id and reason. Invoked by pr-ai-review.yml; never runs PR
code.

An accepted run also sets an "AI PR Review" commit status on the PR head,
linking to the run, so it shows in the PR's checks while it works: the
run itself is a default-branch workflow and would not appear there
otherwise. The status is informational, never a required check.
"""

import json
import os
import re
import sys
import urllib.error
import urllib.request
from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, Mapping, Optional

from .models import EFFORTS

# The whole comment, apart from surrounding whitespace, must be the command
COMMAND_RE = re.compile(r'!ai-review(?P<args>(?:[ \t]+\S+)*)')
ARG_RE = re.compile(r'^(model|effort)=([a-z0-9-]+)$')
WRITE_PERMISSIONS = ('admin', 'maintain', 'write')
STATUS_CONTEXT = 'AI PR Review'
# review job result -> (commit status state, description)
OUTCOMES = {
    'success': ('success', 'Review finished; posted on the PR'),
    'failure': ('error', 'Review failed; see the run'),
    'cancelled': ('error', 'Review cancelled'),
}

Api = Callable[[str, str, Optional[Dict[str, Any]]], Any]


@dataclass(frozen=True)
class Decision:
    run: bool
    reason: str
    pr: int = 0
    head_sha: str = ''
    model: str = 'default'
    effort: str = 'default'
    comment_id: int = 0

    def outputs(self) -> Dict[str, str]:
        return {'run': 'true' if self.run else 'false',
                'reason': self.reason, 'pr': str(self.pr),
                'head_sha': self.head_sha,
                'model': self.model, 'effort': self.effort,
                'comment_id': str(self.comment_id)}


def parse_command(body: str, aliases: Any) -> Optional[Dict[str, str]]:
    """Return {'model', 'effort'} for a !ai-review comment, else None.

    The command must be the entire comment, so mentioning it in a sentence,
    a quote or a longer review never starts a run. Arguments are optional
    `model=<alias>` and `effort=<level>`; anything else is not a command,
    so a typo never silently starts a default review.
    """
    m = COMMAND_RE.fullmatch((body or '').strip())
    if not m:
        return None
    req = {'model': 'default', 'effort': 'default'}
    for token in m.group('args').split():
        a = ARG_RE.match(token)
        if not a:
            return None
        key, value = a.groups()
        if key == 'model' and value not in aliases:
            return None
        if key == 'effort' and value not in EFFORTS:
            return None
        req[key] = value
    return req


def decide(event_name: str, event: Mapping[str, Any],
           inputs: Mapping[str, str], aliases: Any, api: Api,
           repo: str) -> Decision:
    d = _decide(event_name, event, inputs, aliases, api, repo)
    if not d.run:
        return d
    pull = api('GET', f'repos/{repo}/pulls/{d.pr}', None) or {}
    return replace(d, head_sha=(pull.get('head') or {}).get('sha') or '')


def _decide(event_name: str, event: Mapping[str, Any],
            inputs: Mapping[str, str], aliases: Any, api: Api,
            repo: str) -> Decision:
    if event_name == 'workflow_dispatch':
        try:
            pr = int(inputs.get('pr_number', ''))
        except ValueError:
            return Decision(False, 'pr_number is not a number')
        return Decision(True, 'dispatched', pr=pr,
                        model=inputs.get('model') or 'default',
                        effort=inputs.get('effort') or 'default')

    if event_name != 'issue_comment':
        return Decision(False, f'unsupported event {event_name}')
    issue = event.get('issue') or {}
    comment = event.get('comment') or {}
    if not issue.get('pull_request'):
        return Decision(False, 'comment is not on a pull request')
    if issue.get('state') != 'open':
        return Decision(False, 'pull request is not open')
    req = parse_command(comment.get('body') or '', aliases)
    if req is None:
        return Decision(False, 'no !ai-review command')
    user = comment.get('user') or {}
    login = user.get('login') or ''
    if not login or user.get('type') == 'Bot':
        return Decision(False, 'commenter is a bot or unknown')
    perm = api('GET', f'repos/{repo}/collaborators/{login}/permission', None)
    level = (perm or {}).get('permission')
    if level not in WRITE_PERMISSIONS:
        return Decision(False, f'{login} has {level or "no"} access; write '
                               f'access is required')
    return Decision(True, f'requested by {login} ({level})',
                    pr=int(issue['number']), model=req['model'],
                    effort=req['effort'],
                    comment_id=int(comment.get('id') or 0))


def write_outputs(decision: Decision, path: Optional[str]) -> None:
    lines = [f'{k}={v}' for k, v in decision.outputs().items()]
    if path:
        with open(path, 'a', encoding='utf-8') as fh:
            fh.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


def _client_api() -> Api:
    """The few GitHub REST calls this module needs, stdlib only."""
    token = os.environ['GITHUB_TOKEN']

    def api(method: str, path: str, body: Optional[Dict[str, Any]]) -> Any:
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(
            f'https://api.github.com/{path}', data=data, method=method,
            headers={'Authorization': f'Bearer {token}',
                     'Accept': 'application/vnd.github+json',
                     'X-GitHub-Api-Version': '2022-11-28',
                     'User-Agent': 'px4-ai-review',
                     'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                raw = resp.read()
        except urllib.error.HTTPError as e:
            raise RuntimeError(f'{method} {path}: HTTP {e.code} '
                               f'{e.read().decode(errors="replace")[:300]}')
        return json.loads(raw) if raw else None
    return api


def react(api: Api, repo: str, comment_id: int, content: str) -> None:
    """Best-effort reaction on the triggering comment; never fails a run."""
    if not comment_id:
        return
    try:
        api('POST', f'repos/{repo}/issues/comments/{comment_id}/reactions',
            {'content': content})
    except RuntimeError as e:
        print(f'warning: could not react to the comment: {e}',
              file=sys.stderr)


def set_status(api: Api, repo: str, sha: str, state: str, description: str,
               run_url: str, only_if_ours: bool = False) -> None:
    """Best-effort commit status on the PR head; never fails a run.

    With only_if_ours, the status is left alone when the latest one for
    this context links to another run: a newer review of the same commit
    has taken over, and this run being cancelled must not overwrite it.
    """
    if not sha:
        return
    try:
        if only_if_ours:
            latest = next((st for st in api(
                'GET', f'repos/{repo}/commits/{sha}/statuses', None) or []
                if st.get('context') == STATUS_CONTEXT), None)
            if latest and latest.get('target_url') != run_url:
                return
        api('POST', f'repos/{repo}/statuses/{sha}',
            {'state': state, 'context': STATUS_CONTEXT,
             'description': description[:140], 'target_url': run_url})
    except RuntimeError as e:
        print(f'warning: could not set the commit status: {e}',
              file=sys.stderr)


def _run_url(repo: str) -> str:
    server = os.environ.get('GITHUB_SERVER_URL', 'https://github.com')
    return f'{server}/{repo}/actions/runs/{os.environ["GITHUB_RUN_ID"]}'


def main(argv: Any = None) -> int:
    from .models import load
    args = sys.argv[1:] if argv is None else argv
    repo = os.environ['GITHUB_REPOSITORY']
    if args and args[0] == 'react':
        # react <comment_id> <content>
        react(_client_api(), repo, int(args[1] or 0), args[2])
        return 0
    if args and args[0] == 'status':
        # status <sha> <result of the review job>
        state, description = OUTCOMES.get(args[2], OUTCOMES['failure'])
        set_status(_client_api(), repo, args[1], state, description,
                   _run_url(repo), only_if_ours=True)
        return 0
    with open(os.environ['GITHUB_EVENT_PATH'], encoding='utf-8') as fh:
        event = json.load(fh)
    inputs = {k: os.environ.get(f'INPUT_{k.upper()}', '')
              for k in ('pr_number', 'model', 'effort')}
    decision = decide(os.environ.get('GITHUB_EVENT_NAME', ''), event,
                      inputs, load().models, _client_api(), repo)
    write_outputs(decision, os.environ.get('GITHUB_OUTPUT'))
    if decision.run:
        react(_client_api(), repo, decision.comment_id, 'eyes')
        set_status(_client_api(), repo, decision.head_sha, 'pending',
                   f'Reviewing: {decision.reason}', _run_url(repo))
    return 0
