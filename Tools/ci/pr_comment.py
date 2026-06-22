#!/usr/bin/env python3
"""Post, update, or delete a PR comment with deduplication.

Uses hidden HTML markers to find existing comments and avoid duplicates.
Reads comment body from stdin when posting or updating.

Usage:
    echo "comment body" | python3 pr_comment.py --marker pr-title --pr 123 --result fail
    python3 pr_comment.py --marker pr-title --pr 123 --result pass

Results:
    fail  - post/update comment with body from stdin
    warn  - post/update comment with body from stdin
    pass  - delete existing comment if any

Requires GH_TOKEN and GITHUB_REPOSITORY environment variables.
"""

import argparse
import json
import os
import subprocess
import sys


def gh_api(endpoint: str, method: str = 'GET', body: dict | None = None) -> str:
    """Call the GitHub API via gh cli."""
    cmd = ['gh', 'api', endpoint, '-X', method]
    if body:
        for key, value in body.items():
            cmd.extend(['-f', f'{key}={value}'])
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0 and method != 'DELETE':
        print(f"gh api error: {result.stderr}", file=sys.stderr)
    return result.stdout


def find_comment(repo: str, pr: int, marker: str) -> str | None:
    """Find an existing comment by its hidden marker. Returns comment ID or None."""
    response = gh_api(f"repos/{repo}/issues/{pr}/comments?per_page=100")
    try:
        comments = json.loads(response)
    except json.JSONDecodeError:
        return None

    if not isinstance(comments, list):
        return None

    for comment in comments:
        if isinstance(comment, dict) and comment.get('body', '').startswith(marker):
            return str(comment['id'])
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description='Manage PR quality comments')
    parser.add_argument('--marker', required=True,
                        help='Marker name (e.g. pr-title, commit-msgs, pr-body)')
    parser.add_argument('--pr', required=True, type=int,
                        help='Pull request number')
    parser.add_argument('--result', required=True, choices=['pass', 'fail', 'warn'],
                        help='Check result: pass deletes comment, fail/warn posts it')
    args = parser.parse_args()

    repo = os.environ.get('GITHUB_REPOSITORY', '')
    if not repo:
        print("GITHUB_REPOSITORY not set", file=sys.stderr)
        sys.exit(2)

    marker = f"<!-- commit-quality-{args.marker} -->"
    existing_id = find_comment(repo, args.pr, marker)

    if args.result == 'pass':
        if existing_id:
            gh_api(f"repos/{repo}/issues/comments/{existing_id}", method='DELETE')
        return

    # Read comment body from stdin
    body_content = sys.stdin.read().strip()
    if not body_content:
        print("No comment body provided on stdin", file=sys.stderr)
        sys.exit(2)

    full_body = f"{marker}\n{body_content}"

    if existing_id:
        gh_api(f"repos/{repo}/issues/comments/{existing_id}", method='PATCH',
               body={'body': full_body})
    else:
        gh_api(f"repos/{repo}/issues/{args.pr}/comments", method='POST',
               body={'body': full_body})


if __name__ == '__main__':
    main()
