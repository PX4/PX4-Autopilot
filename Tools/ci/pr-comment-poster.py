#!/usr/bin/env python3
"""
PR comment poster for analysis workflows.

This script is invoked from the `PR Comment Poster` workflow which runs on
`workflow_run` in the base repository context. It consumes a `pr-comment`
artifact produced by an upstream analysis job (clang-tidy, flash_analysis,
etc.) and posts or updates a sticky PR comment via the GitHub REST API.

Artifact contract (directory passed on the command line):

  manifest.json
    {
      "pr_number": 12345,                                     (required, int > 0)
      "marker":    "<!-- pr-comment-poster:flash-analysis -->", (required, printable ASCII)
      "mode":      "upsert"                                    (optional, default "upsert")
    }

  body.md
    Markdown comment body, posted verbatim. Must be non-empty and
    <= 60000 bytes (GitHub's hard limit is 65535, we cap under).

Security: this script is run in a write-token context from a workflow that
MUST NOT check out PR code. Both manifest.json and body.md are treated as
opaque data. The marker is validated to printable ASCII only before use.

Subcommands:

  validate <dir>   Validate that <dir> contains a conforming manifest + body.
  post <dir>       Validate, then upsert a sticky comment on the target PR.
                   Requires env GITHUB_TOKEN and GITHUB_REPOSITORY.

Python stdlib only. No third-party dependencies.
"""

import argparse
import json
import os
import sys
import typing
import urllib.error
import urllib.request


# GitHub hard limit is 65535 bytes. Cap well under to leave headroom for
# the appended marker line and any future wrapping.
MAX_BODY_BYTES = 60000

# Marker length bounds. 1..200 is plenty for an HTML comment tag such as
# "<!-- pr-comment-poster:flash-analysis -->".
MARKER_MIN_LEN = 1
MARKER_MAX_LEN = 200

ACCEPTED_MODES = ('upsert',)

GITHUB_API = 'https://api.github.com'
USER_AGENT = 'px4-pr-comment-poster'


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def _fail(msg: str) -> typing.NoReturn:
    print('error: {}'.format(msg), file=sys.stderr)
    sys.exit(1)


def _is_printable_ascii(s):
    # Space (0x20) through tilde (0x7E) inclusive.
    return all(0x20 <= ord(ch) <= 0x7E for ch in s)


def validate_marker(marker):
    """Validate the marker string.

    The marker is printable ASCII only and bounded in length. The original
    shell implementation also rejected quotes, backticks, and backslashes
    because the value flowed through jq and shell contexts. Now that Python
    owns the handling (the value is only ever used as a substring match in
    comment bodies and as a literal string in JSON request payloads that
    urllib serialises for us) those characters are safe. We keep the
    printable-ASCII and length rules as a belt-and-braces sanity check.
    """
    if not isinstance(marker, str):
        _fail('marker must be a string')
    n = len(marker)
    if n < MARKER_MIN_LEN or n > MARKER_MAX_LEN:
        _fail('marker length out of range ({}..{}): {}'.format(
            MARKER_MIN_LEN, MARKER_MAX_LEN, n))
    if not _is_printable_ascii(marker):
        _fail('marker contains non-printable or non-ASCII character')


def validate_manifest(directory):
    """Validate <directory>/manifest.json and <directory>/body.md.

    Returns a dict with keys: pr_number (int), marker (str), mode (str),
    body (str, verbatim contents of body.md).
    """
    manifest_path = os.path.join(directory, 'manifest.json')
    body_path = os.path.join(directory, 'body.md')

    if not os.path.isfile(manifest_path):
        _fail('manifest.json missing at {}'.format(manifest_path))
    if not os.path.isfile(body_path):
        _fail('body.md missing at {}'.format(body_path))

    try:
        with open(manifest_path, 'r', encoding='utf-8') as f:
            manifest = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        _fail('manifest.json is not valid JSON: {}'.format(e))

    if not isinstance(manifest, dict):
        _fail('manifest.json must be a JSON object')

    pr_number = manifest.get('pr_number')
    # bool is a subclass of int in Python, so isinstance(True, int) is True.
    # Reject bools explicitly so "true"/"false" in the manifest doesn't silently
    # validate as 1/0 and then either fail upstream or poke the wrong PR.
    if not isinstance(pr_number, int) or isinstance(pr_number, bool):
        _fail('pr_number must be an integer')
    if pr_number <= 0:
        _fail('pr_number must be > 0 (got {})'.format(pr_number))

    marker = manifest.get('marker')
    validate_marker(marker)

    mode = manifest.get('mode', 'upsert')
    if mode not in ACCEPTED_MODES:
        _fail('unsupported mode {!r} (accepted: {})'.format(
            mode, ', '.join(ACCEPTED_MODES)))

    # Read as bytes first so the size check is an honest byte count (matching
    # GitHub's own 65535-byte comment limit) before we pay the cost of decoding.
    try:
        with open(body_path, 'rb') as f:
            body_bytes = f.read()
    except OSError as e:
        _fail('could not read body.md: {}'.format(e))

    if len(body_bytes) == 0:
        _fail('body.md is empty')
    if len(body_bytes) > MAX_BODY_BYTES:
        _fail('body.md too large: {} bytes (max {})'.format(
            len(body_bytes), MAX_BODY_BYTES))

    # Require UTF-8 up front so a producer that wrote a garbage encoding fails
    # here rather than later inside json.dumps with a less obvious traceback.
    try:
        body = body_bytes.decode('utf-8')
    except UnicodeDecodeError as e:
        _fail('body.md is not valid UTF-8: {}'.format(e))

    return {
        'pr_number': pr_number,
        'marker': marker,
        'mode': mode,
        'body': body,
    }


# ---------------------------------------------------------------------------
# GitHub HTTP helpers
# ---------------------------------------------------------------------------

def _github_request(method, url, token, json_body=None):
    """Perform a single GitHub REST request.

    Returns a tuple (parsed_json_or_none, headers_dict). Raises RuntimeError
    with the server response body on HTTP errors so CI logs show what
    GitHub complained about.
    """
    data = None
    headers = {
        'Authorization': 'Bearer {}'.format(token),
        'Accept': 'application/vnd.github+json',
        # Pin the API version so GitHub deprecations don't silently change
        # the response shape under us.
        'X-GitHub-Api-Version': '2022-11-28',
        'User-Agent': USER_AGENT,
    }
    if json_body is not None:
        data = json.dumps(json_body).encode('utf-8')
        headers['Content-Type'] = 'application/json; charset=utf-8'

    req = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req) as resp:
            raw = resp.read()
            # HTTPMessage is case-insensitive on lookup but its items() preserves
            # the original case. GitHub sends "Link" with a capital L, which is
            # what _parse_next_link expects.
            resp_headers = dict(resp.headers.items())
            if not raw:
                return None, resp_headers
            return json.loads(raw.decode('utf-8')), resp_headers
    except urllib.error.HTTPError as e:
        # GitHub error bodies are JSON with a "message" field and often a
        # "documentation_url". Dump the raw body into the exception so the CI
        # log shows exactly what the API objected to. A bare "HTTP 422"
        # tells us nothing useful.
        try:
            err_body = e.read().decode('utf-8', errors='replace')
        except Exception:
            err_body = '(no body)'
        raise RuntimeError(
            'GitHub API {} {} failed: HTTP {} {}\n{}'.format(
                method, url, e.code, e.reason, err_body))
    except urllib.error.URLError as e:
        # Network layer failure (DNS, TLS, connection reset). No HTTP response
        # to parse; just surface the transport reason.
        raise RuntimeError(
            'GitHub API {} {} failed: {}'.format(method, url, e.reason))


def _parse_next_link(link_header):
    """Return the URL for rel="next" from an RFC 5988 Link header, or None.

    The Link header is comma-separated entries of the form:
      <https://...?page=2>; rel="next", <https://...?page=5>; rel="last"
    We walk each entry and return the URL of the one whose rel attribute is
    "next". Accept single-quoted rel values for robustness even though GitHub
    always emits double quotes.
    """
    if not link_header:
        return None
    for part in link_header.split(','):
        segs = part.strip().split(';')
        if len(segs) < 2:
            continue
        url_seg = segs[0].strip()
        if not (url_seg.startswith('<') and url_seg.endswith('>')):
            continue
        url = url_seg[1:-1]
        for attr in segs[1:]:
            attr = attr.strip()
            if attr == 'rel="next"' or attr == "rel='next'":
                return url
    return None


def github_api(method, path, token, json_body=None):
    """GET/POST/PATCH a single GitHub API path. Non-paginated."""
    url = '{}/{}'.format(GITHUB_API.rstrip('/'), path.lstrip('/'))
    body, _ = _github_request(method, url, token, json_body=json_body)
    return body


def github_api_paginated(path, token):
    """GET a GitHub API path and follow rel="next" Link headers.

    Yields items from each page's JSON array.
    """
    url = '{}/{}'.format(GITHUB_API.rstrip('/'), path.lstrip('/'))
    # GitHub defaults to per_page=30. Bump to 100 (the max) so a PR with a
    # few hundred comments fetches in 3 or 4 round-trips instead of 10+.
    sep = '&' if '?' in url else '?'
    url = '{}{}per_page=100'.format(url, sep)
    while url is not None:
        body, headers = _github_request('GET', url, token)
        if body is None:
            return
        if not isinstance(body, list):
            raise RuntimeError(
                'expected JSON array from {}, got {}'.format(
                    url, type(body).__name__))
        for item in body:
            yield item
        url = _parse_next_link(headers.get('Link'))


# ---------------------------------------------------------------------------
# Comment upsert
# ---------------------------------------------------------------------------

def find_existing_comment_id(token, repo, pr_number, marker):
    """Return the id of the first PR comment whose body contains marker, or None.

    PR comments are issue comments in GitHub's data model, so we hit
    /issues/{n}/comments rather than /pulls/{n}/comments (the latter only
    returns review comments tied to specific code lines, which is not what
    we want). The match is a plain substring check against the comment body;
    the marker is expected to be an HTML comment that will not accidentally
    appear in user-written prose.
    """
    path = 'repos/{}/issues/{}/comments'.format(repo, pr_number)
    for comment in github_api_paginated(path, token):
        body = comment.get('body') or ''
        if marker in body:
            return comment.get('id')
    return None


def build_final_body(body, marker):
    """Append the marker to body if not already present.

    If the caller already embedded the marker (e.g. inside a hidden HTML
    comment anywhere in their body) we leave the body alone; otherwise we
    rstrip trailing newlines and append the marker on its own line after a
    blank-line separator. Trailing-newline stripping keeps the output from
    accumulating extra blank lines every time an existing comment is
    re-rendered and re-posted.
    """
    if marker in body:
        return body
    return '{}\n\n{}\n'.format(body.rstrip('\n'), marker)


def upsert_comment(token, repo, pr_number, marker, body):
    final_body = build_final_body(body, marker)
    existing_id = find_existing_comment_id(token, repo, pr_number, marker)

    if existing_id is not None:
        print('Updating comment {} on PR #{}'.format(existing_id, pr_number))
        github_api(
            'PATCH',
            'repos/{}/issues/comments/{}'.format(repo, existing_id),
            token,
            json_body={'body': final_body},
        )
    else:
        print('Creating new comment on PR #{}'.format(pr_number))
        github_api(
            'POST',
            'repos/{}/issues/{}/comments'.format(repo, pr_number),
            token,
            json_body={'body': final_body},
        )


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def cmd_validate(args):
    result = validate_manifest(args.directory)
    print('ok: pr_number={} marker_len={} mode={} body_bytes={}'.format(
        result['pr_number'],
        len(result['marker']),
        result['mode'],
        len(result['body'].encode('utf-8')),
    ))
    return 0


def cmd_post(args):
    result = validate_manifest(args.directory)

    # GITHUB_TOKEN is provided by the workflow via env; GITHUB_REPOSITORY is
    # auto-set on every Actions runner. Both are required here because a local
    # developer running the script directly won't have either unless they
    # export them, and we want a clear error in that case.
    token = os.environ.get('GITHUB_TOKEN')
    if not token:
        _fail('GITHUB_TOKEN is not set')
    repo = os.environ.get('GITHUB_REPOSITORY')
    if not repo:
        _fail('GITHUB_REPOSITORY is not set (expected "owner/name")')
    # Minimal shape check. If "owner/name" is malformed the subsequent API
    # calls would 404 with an unhelpful URL. Fail fast here instead.
    if '/' not in repo:
        _fail('GITHUB_REPOSITORY must be "owner/name", got {!r}'.format(repo))

    try:
        upsert_comment(
            token=token,
            repo=repo,
            pr_number=result['pr_number'],
            marker=result['marker'],
            body=result['body'],
        )
    except RuntimeError as e:
        _fail(str(e))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Validate and post sticky PR comments from CI artifacts.',
    )
    sub = parser.add_subparsers(dest='command', required=True)

    p_validate = sub.add_parser(
        'validate',
        help='Validate manifest.json and body.md in the given directory.',
    )
    p_validate.add_argument('directory')
    p_validate.set_defaults(func=cmd_validate)

    p_post = sub.add_parser(
        'post',
        help='Validate, then upsert a sticky PR comment. Requires env '
             'GITHUB_TOKEN and GITHUB_REPOSITORY.',
    )
    p_post.add_argument('directory')
    p_post.set_defaults(func=cmd_post)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
