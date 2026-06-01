#!/usr/bin/env python3
"""
PR review-comment poster for analysis workflows.

Sibling of Tools/ci/pr-comment-poster.py. Where pr-comment-poster.py posts
sticky issue-style PR comments, this script posts line-anchored review
comments on the "Files changed" tab. Use it for tools like clang-tidy that
want to flag specific lines instead of (or in addition to) a rollup
comment.

This script is invoked from the `PR Review Poster` workflow which runs on
`workflow_run` in the base repository context. It consumes a `pr-review`
artifact produced by an upstream analysis job and posts a fresh PR review
via the GitHub REST API, dismissing any stale review the same producer
left on a previous run.

Artifact contract (directory passed on the command line):

  manifest.json
    {
      "pr_number":  12345,                                    (required, int > 0)
      "marker":     "<!-- pr-review-poster:clang-tidy -->",   (required, printable ASCII)
      "event":      "COMMENT",                                 (required, "COMMENT" only)
      "commit_sha": "0123456789abcdef0123456789abcdef01234567",(required, 40 hex chars)
      "summary":    "Optional review body text"                (optional)
    }

  comments.json
    JSON array of line-anchored review comment objects:
      [
        {"path": "src/foo.cpp", "line": 42, "side": "RIGHT",
         "body": "..."},
        {"path": "src/bar.hpp", "start_line": 10, "line": 15,
         "side": "RIGHT", "start_side": "RIGHT", "body": "..."}
      ]

Note: `APPROVE` and `REQUEST_CHANGES` events are intentionally NOT
supported. Bots should never approve a pull request, and REQUEST_CHANGES
cannot be dismissed by the GITHUB_TOKEN when branch protection restricts
review dismissals, leading to undismissable spam on every push.

Security: this script is run in a write-token context from a workflow that
MUST NOT check out PR code. Both manifest.json and comments.json are
treated as opaque data. The marker is validated to printable ASCII only
before use, and only reviews authored by github-actions[bot] whose body
contains the marker can be dismissed (a fork cannot spoof either).

Subcommands:

  validate <dir>   Validate that <dir> contains a conforming manifest +
                   comments file.
  post <dir>       Validate, then dismiss any stale matching review and
                   post a new review on the target PR. Requires env
                   GITHUB_TOKEN and GITHUB_REPOSITORY.

Python stdlib only. No third-party dependencies.
"""

import argparse
import json
import os
import re
import sys
import time

import _github_helpers
from _github_helpers import fail as _fail


USER_AGENT = 'px4-pr-review-poster'

# Marker length bounds. 1..200 is plenty for an HTML comment tag such as
# "<!-- pr-review-poster:clang-tidy -->".
MARKER_MIN_LEN = 1
MARKER_MAX_LEN = 200

# Cap per-comment body size well under GitHub's hard limit so we leave
# headroom for the wrapping JSON envelope. Empirically GitHub allows ~65535
# bytes per review comment body; 60000 is a safe ceiling.
MAX_COMMENT_BODY_BYTES = 60000

# Cap on number of comments per single review POST. platisd uses 10. The
# value matters because GitHub's review-creation endpoint has a payload
# size limit and review comments occasionally trip an abuse-detection
# threshold when posted in very large batches. Smaller chunks also let us
# spread the work across multiple reviews so a single bad entry only
# fails its own chunk.
COMMENTS_PER_REVIEW = 10

# Sleep between successive review POSTs to stay clear of GitHub's
# secondary rate limits. platisd uses 10s; 5s is enough for our volume
# and cuts user-visible latency.
SLEEP_BETWEEN_CHUNKS_SECONDS = 5

ACCEPTED_EVENTS = ('COMMENT',)
ACCEPTED_SIDES = ('LEFT', 'RIGHT')
COMMIT_SHA_RE = re.compile(r'^[0-9a-f]{40}$')

# The login GitHub assigns to the built-in actions token. Used to filter
# the list of existing reviews so we never touch a human reviewer's review.
BOT_LOGIN = 'github-actions[bot]'


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def _is_printable_ascii(s):
    return all(0x20 <= ord(ch) <= 0x7E for ch in s)


def validate_marker(marker):
    """Validate the marker string. See pr-comment-poster.py for rationale."""
    if not isinstance(marker, str):
        _fail('marker must be a string')
    n = len(marker)
    if n < MARKER_MIN_LEN or n > MARKER_MAX_LEN:
        _fail('marker length out of range ({}..{}): {}'.format(
            MARKER_MIN_LEN, MARKER_MAX_LEN, n))
    if not _is_printable_ascii(marker):
        _fail('marker contains non-printable or non-ASCII character')


def _validate_comment_entry(idx, entry):
    """Validate a single review-comment entry. Raises via _fail on error."""
    if not isinstance(entry, dict):
        _fail('comments[{}]: must be an object'.format(idx))

    path = entry.get('path')
    if not isinstance(path, str) or not path:
        _fail('comments[{}].path: required non-empty string'.format(idx))

    line = entry.get('line')
    if not isinstance(line, int) or isinstance(line, bool) or line <= 0:
        _fail('comments[{}].line: required positive integer'.format(idx))

    side = entry.get('side', 'RIGHT')
    if side not in ACCEPTED_SIDES:
        _fail('comments[{}].side: must be one of {} (got {!r})'.format(
            idx, ', '.join(ACCEPTED_SIDES), side))

    if 'start_line' in entry:
        start_line = entry['start_line']
        if (not isinstance(start_line, int)
                or isinstance(start_line, bool)
                or start_line <= 0):
            _fail('comments[{}].start_line: must be positive integer'.format(idx))
        if start_line >= line:
            _fail('comments[{}].start_line ({}) must be < line ({})'.format(
                idx, start_line, line))
        start_side = entry.get('start_side', side)
        if start_side not in ACCEPTED_SIDES:
            _fail('comments[{}].start_side: must be one of {}'.format(
                idx, ', '.join(ACCEPTED_SIDES)))

    body = entry.get('body')
    if not isinstance(body, str) or not body:
        _fail('comments[{}].body: required non-empty string'.format(idx))
    body_bytes = len(body.encode('utf-8'))
    if body_bytes > MAX_COMMENT_BODY_BYTES:
        _fail('comments[{}].body too large: {} bytes (max {})'.format(
            idx, body_bytes, MAX_COMMENT_BODY_BYTES))


def validate_manifest(directory):
    """Validate <directory>/manifest.json and <directory>/comments.json.

    Returns a dict with keys: pr_number, marker, event, commit_sha,
    summary, comments (list of validated comment dicts).
    """
    manifest_path = os.path.join(directory, 'manifest.json')
    comments_path = os.path.join(directory, 'comments.json')

    if not os.path.isfile(manifest_path):
        _fail('manifest.json missing at {}'.format(manifest_path))
    if not os.path.isfile(comments_path):
        _fail('comments.json missing at {}'.format(comments_path))

    try:
        with open(manifest_path, 'r', encoding='utf-8') as f:
            manifest = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        _fail('manifest.json is not valid JSON: {}'.format(e))

    if not isinstance(manifest, dict):
        _fail('manifest.json must be a JSON object')

    pr_number = manifest.get('pr_number')
    if not isinstance(pr_number, int) or isinstance(pr_number, bool):
        _fail('pr_number must be an integer')
    if pr_number <= 0:
        _fail('pr_number must be > 0 (got {})'.format(pr_number))

    marker = manifest.get('marker')
    validate_marker(marker)

    event = manifest.get('event')
    if event not in ACCEPTED_EVENTS:
        _fail('event must be one of {} (got {!r}). APPROVE and '
              'REQUEST_CHANGES are intentionally forbidden.'.format(
                  ', '.join(ACCEPTED_EVENTS), event))

    commit_sha = manifest.get('commit_sha')
    if not isinstance(commit_sha, str) or not COMMIT_SHA_RE.match(commit_sha):
        _fail('commit_sha must be a 40-character lowercase hex string')

    summary = manifest.get('summary', '')
    if summary is None:
        summary = ''
    if not isinstance(summary, str):
        _fail('summary must be a string if present')

    try:
        with open(comments_path, 'r', encoding='utf-8') as f:
            comments = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        _fail('comments.json is not valid JSON: {}'.format(e))

    if not isinstance(comments, list):
        _fail('comments.json must be a JSON array')

    for idx, entry in enumerate(comments):
        _validate_comment_entry(idx, entry)

    return {
        'pr_number': pr_number,
        'marker': marker,
        'event': event,
        'commit_sha': commit_sha,
        'summary': summary,
        'comments': comments,
    }


# ---------------------------------------------------------------------------
# Stale-review dismissal
# ---------------------------------------------------------------------------

def find_stale_reviews(client, repo, pr_number, marker):
    """Yield (id, state) for each existing review owned by the bot AND
    whose body contains the marker.

    Filtering on BOTH author == github-actions[bot] AND marker-in-body is
    the security invariant: a fork PR cannot impersonate the bot login,
    and a fork PR also cannot inject the marker into a human reviewer's
    body without API access.
    """
    path = 'repos/{}/pulls/{}/reviews'.format(repo, pr_number)
    for review in client.paginated(path):
        user = review.get('user') or {}
        if user.get('login') != BOT_LOGIN:
            continue
        body = review.get('body') or ''
        if marker not in body:
            continue
        yield review.get('id'), review.get('state')


def dismiss_stale_reviews(client, repo, pr_number, marker):
    """Dismiss (or, for PENDING reviews, delete) every stale matching review.

    Returns the number of reviews that could NOT be dismissed (still active).
    """
    dismissal_message = 'Superseded by a newer run'
    failed_dismissals = 0
    for review_id, state in find_stale_reviews(client, repo, pr_number, marker):
        if review_id is None:
            continue
        if state in ('DISMISSED', 'COMMENTED'):
            # Already inert or non-blocking; nothing to do.
            continue
        if state == 'PENDING':
            # PENDING reviews cannot be dismissed; they must be deleted.
            print('Deleting pending stale review {}'.format(review_id))
            try:
                client.request(
                    'DELETE',
                    'repos/{}/pulls/{}/reviews/{}'.format(
                        repo, pr_number, review_id))
            except RuntimeError as e:
                failed_dismissals += 1
                print('warning: failed to delete pending review {}: {}'.format(
                    review_id, e), file=sys.stderr)
            continue
        print('Dismissing stale review {} (state={})'.format(review_id, state))
        try:
            client.request(
                'PUT',
                'repos/{}/pulls/{}/reviews/{}/dismissals'.format(
                    repo, pr_number, review_id),
                json_body={
                    'message': dismissal_message,
                    'event': 'DISMISS',
                },
            )
        except RuntimeError as e:
            failed_dismissals += 1
            print('warning: failed to dismiss review {}: {}'.format(
                review_id, e), file=sys.stderr)
    return failed_dismissals


# ---------------------------------------------------------------------------
# Review posting
# ---------------------------------------------------------------------------

def _chunk(lst, n):
    """Yield successive n-sized slices of lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


def _build_review_body(marker, summary, chunk_index, chunk_total):
    """Construct the review body text.

    Always begins with the marker (so future runs can find and dismiss
    this review). Appends a chunk index when the comment set is split
    across multiple reviews, and the producer-supplied summary if any.
    """
    parts = [marker]
    if chunk_total > 1:
        parts.append('({}/{})'.format(chunk_index + 1, chunk_total))
    if summary:
        parts.append('')
        parts.append(summary)
    return '\n'.join(parts)


def _comment_to_api(entry):
    """Project a validated comment dict to the GitHub API shape."""
    api = {
        'path': entry['path'],
        'line': entry['line'],
        'side': entry.get('side', 'RIGHT'),
        'body': entry['body'],
    }
    if 'start_line' in entry:
        api['start_line'] = entry['start_line']
        api['start_side'] = entry.get('start_side', api['side'])
    return api


def post_review(client, repo, pr_number, marker, event, commit_sha, summary,
                comments):
    """Post one or more reviews containing the validated comments.

    Comments are split into COMMENTS_PER_REVIEW-sized chunks. Each chunk
    becomes its own review POST. A failed chunk logs a warning and the
    loop continues to the next chunk.
    """
    chunks = list(_chunk(comments, COMMENTS_PER_REVIEW))
    total = len(chunks)
    if total == 0:
        print('No comments to post; skipping review creation.')
        return

    posted_any = False
    for idx, chunk in enumerate(chunks):
        if idx > 0:
            time.sleep(SLEEP_BETWEEN_CHUNKS_SECONDS)
        body = _build_review_body(marker, summary, idx, total)
        payload = {
            'commit_id': commit_sha,
            'body': body,
            'event': event,
            'comments': [_comment_to_api(c) for c in chunk],
        }
        print('Posting review chunk {}/{} with {} comment(s)'.format(
            idx + 1, total, len(chunk)))
        try:
            client.request(
                'POST',
                'repos/{}/pulls/{}/reviews'.format(repo, pr_number),
                json_body=payload,
            )
            posted_any = True
        except RuntimeError as e:
            # Most common cause is HTTP 422: a comment refers to a line
            # GitHub does not consider part of the diff. Skip the bad
            # chunk and keep going so other findings still get posted.
            print('warning: review chunk {}/{} failed: {}'.format(
                idx + 1, total, e), file=sys.stderr)

    if not posted_any:
        # If every single chunk failed, surface that as a hard error so
        # the workflow turns red and a human looks at it.
        _fail('all review chunks failed to post; see warnings above')


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def cmd_validate(args):
    result = validate_manifest(args.directory)
    print(('ok: pr_number={} marker_len={} event={} commit_sha={} '
           'comments={} summary_len={}').format(
        result['pr_number'],
        len(result['marker']),
        result['event'],
        result['commit_sha'],
        len(result['comments']),
        len(result['summary']),
    ))
    return 0


def cmd_post(args):
    result = validate_manifest(args.directory)

    # Empty comment lists short-circuit silently. A producer that ran but
    # found nothing to flag should not generate noise on the PR.
    if len(result['comments']) == 0:
        print('No comments in artifact; nothing to post.')
        return 0

    token = os.environ.get('GITHUB_TOKEN')
    if not token:
        _fail('GITHUB_TOKEN is not set')
    repo = os.environ.get('GITHUB_REPOSITORY')
    if not repo:
        _fail('GITHUB_REPOSITORY is not set (expected "owner/name")')
    if '/' not in repo:
        _fail('GITHUB_REPOSITORY must be "owner/name", got {!r}'.format(repo))

    try:
        client = _github_helpers.GitHubClient(token, user_agent=USER_AGENT)
        undismissed = dismiss_stale_reviews(
            client=client,
            repo=repo,
            pr_number=result['pr_number'],
            marker=result['marker'],
        )

        if undismissed > 0:
            print('{} prior review(s) could not be dismissed (likely '
                  'branch protection).'.format(undismissed))

        post_review(
            client=client,
            repo=repo,
            pr_number=result['pr_number'],
            marker=result['marker'],
            event=result['event'],
            commit_sha=result['commit_sha'],
            summary=result['summary'],
            comments=result['comments'],
        )
    except RuntimeError as e:
        _fail(str(e))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Validate and post line-anchored PR review comments '
                    'from CI artifacts.',
    )
    sub = parser.add_subparsers(dest='command', required=True)

    p_validate = sub.add_parser(
        'validate',
        help='Validate manifest.json and comments.json in the given directory.',
    )
    p_validate.add_argument('directory')
    p_validate.set_defaults(func=cmd_validate)

    p_post = sub.add_parser(
        'post',
        help='Validate, then dismiss any stale review and post a new one. '
             'Requires env GITHUB_TOKEN and GITHUB_REPOSITORY.',
    )
    p_post.add_argument('directory')
    p_post.set_defaults(func=cmd_post)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
