#!/usr/bin/env python3
"""
Shared GitHub REST helpers for PX4 CI scripts.

This module is imported by the PR poster scripts under Tools/ci/. It is
NOT an executable entry point; do not run it directly.

Provides:
  - fail(msg)              terminates the caller with a clear error
  - GitHubClient(token)    thin stdlib-only GitHub REST client with
                           single-request and paginated helpers

Python stdlib only. No third-party dependencies.

History: extracted from Tools/ci/pr-comment-poster.py so that
pr-comment-poster.py and pr-review-poster.py share the same HTTP plumbing
without duplicating ~100 lines of request/pagination/error-handling code.
"""

import json
import sys
import typing
import urllib.error
import urllib.request


GITHUB_API = 'https://api.github.com'
DEFAULT_USER_AGENT = 'px4-ci'
API_VERSION = '2022-11-28'


def fail(msg: str) -> typing.NoReturn:
    """Print an error to stderr and exit with status 1.

    Annotated NoReturn so static checkers understand control does not
    continue past a fail() call.
    """
    print('error: {}'.format(msg), file=sys.stderr)
    sys.exit(1)


def _parse_next_link(link_header):
    """Return the URL for rel="next" from an RFC 5988 Link header, or None.

    The Link header is comma-separated entries of the form:
      <https://...?page=2>; rel="next", <https://...?page=5>; rel="last"
    We walk each entry and return the URL of the one whose rel attribute is
    "next". Accept single-quoted rel values for robustness even though
    GitHub always emits double quotes.
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


class GitHubClient:
    """Minimal GitHub REST client backed by the Python stdlib.

    Each instance holds a token and a user-agent so callers do not have to
    thread them through every call. Methods return parsed JSON (or None for
    empty responses) and raise RuntimeError with the server response body on
    HTTP errors, so CI logs show what the API actually objected to.

    Usage:
        client = GitHubClient(token, user_agent='px4-pr-comment-poster')
        body, headers = client.request('GET', 'repos/{o}/{r}/pulls/123')
        for item in client.paginated('repos/{o}/{r}/pulls/123/reviews'):
            ...
    """

    def __init__(self, token, user_agent=DEFAULT_USER_AGENT):
        if not token:
            raise ValueError('GitHub token is required')
        self._token = token
        self._user_agent = user_agent

    def request(self, method, path_or_url, json_body=None):
        """GET/POST/PATCH/PUT/DELETE a single API path or absolute URL.

        `path_or_url` may be either a relative API path (e.g.
        "repos/PX4/PX4-Autopilot/pulls/123") or an absolute URL such as the
        next-page URL returned from paginated results. Relative paths are
        prefixed with the GitHub API base.

        Returns (parsed_json_or_none, headers_dict). Raises RuntimeError
        on HTTP or transport errors.
        """
        url = self._resolve(path_or_url)
        return self._do_request(method, url, json_body)

    def paginated(self, path, per_page=100):
        """GET a path and follow rel="next" Link headers.

        Yields items from each page's JSON array. Bumps per_page to 100
        (GitHub's max) so large result sets take fewer round-trips.
        Raises RuntimeError if any page response is not a JSON array.
        """
        url = self._resolve(path)
        sep = '&' if '?' in url else '?'
        url = '{}{}per_page={}'.format(url, sep, per_page)
        while url is not None:
            body, headers = self._do_request('GET', url, None)
            if body is None:
                return
            if not isinstance(body, list):
                raise RuntimeError(
                    'expected JSON array from {}, got {}'.format(
                        url, type(body).__name__))
            for item in body:
                yield item
            url = _parse_next_link(headers.get('Link'))

    def _resolve(self, path_or_url):
        if path_or_url.startswith('http://') or path_or_url.startswith('https://'):
            return path_or_url
        return '{}/{}'.format(GITHUB_API.rstrip('/'), path_or_url.lstrip('/'))

    def _do_request(self, method, url, json_body):
        data = None
        headers = {
            'Authorization': 'Bearer {}'.format(self._token),
            'Accept': 'application/vnd.github+json',
            # Pin the API version so GitHub deprecations don't silently
            # change the response shape under us.
            'X-GitHub-Api-Version': API_VERSION,
            'User-Agent': self._user_agent,
        }
        if json_body is not None:
            data = json.dumps(json_body).encode('utf-8')
            headers['Content-Type'] = 'application/json; charset=utf-8'

        req = urllib.request.Request(
            url, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req) as resp:
                raw = resp.read()
                # HTTPMessage is case-insensitive on lookup but its items()
                # preserves the original case. GitHub sends "Link" with a
                # capital L, which is what _parse_next_link expects.
                resp_headers = dict(resp.headers.items())
                if not raw:
                    return None, resp_headers
                return json.loads(raw.decode('utf-8')), resp_headers
        except urllib.error.HTTPError as e:
            # GitHub error bodies are JSON with a "message" field and often
            # a "documentation_url". Dump the raw body into the exception so
            # the CI log shows exactly what the API objected to. A bare
            # "HTTP 422" tells us nothing useful.
            try:
                err_body = e.read().decode('utf-8', errors='replace')
            except Exception:
                err_body = '(no body)'
            raise RuntimeError(
                'GitHub API {} {} failed: HTTP {} {}\n{}'.format(
                    method, url, e.code, e.reason, err_body))
        except urllib.error.URLError as e:
            # Network layer failure (DNS, TLS, connection reset). No HTTP
            # response to parse; just surface the transport reason.
            raise RuntimeError(
                'GitHub API {} {} failed: {}'.format(method, url, e.reason))
