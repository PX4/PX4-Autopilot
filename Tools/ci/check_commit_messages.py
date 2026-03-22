#!/usr/bin/env python3
"""Validate commit messages in a PR against conventional commits format.

Reads a JSON array of GitHub commit objects from stdin (as returned by the
GitHub API's /pulls/{n}/commits endpoint) and checks each message for
blocking errors and advisory warnings.

With --markdown, outputs a formatted PR comment body instead of plain text.
"""

import json
import sys

from conventional_commits import (
    EXEMPT_PREFIXES,
    parse_header,
)

# Blocking: prefixes that indicate unsquashed fixup commits
FIXUP_PREFIXES = ('fixup!', 'squash!', 'amend!')

# Blocking: single-word throwaway messages (case-insensitive exact match)
THROWAWAY_WORDS = frozenset({
    'fix', 'fixed', 'fixes',
    'update', 'updated', 'updates',
    'test', 'tests', 'testing',
    'tmp', 'temp',
    'oops', 'wip',
    'debug', 'cleanup',
})

# Blocking: debug session leftovers
DEBUG_KEYWORDS = ('tmate',)

# Warning: review-response messages (case-insensitive substring match)
REVIEW_RESPONSE_PATTERNS = (
    'address review',
    'apply suggestions from code review',
    'code review',
)

# Warning: formatter-only commits
FORMATTER_PATTERNS = (
    'do make format',
    'make format',
    'run formatter',
    'apply format',
)

MIN_MESSAGE_LENGTH = 5


def check_commit(message: str) -> tuple[list[str], list[str]]:
    """Return (errors, warnings) for a single commit message."""
    errors: list[str] = []
    warnings: list[str] = []

    first_line = message.split('\n', 1)[0].strip()
    lower = first_line.lower()

    # --- Blocking checks ---

    for prefix in FIXUP_PREFIXES:
        if lower.startswith(prefix):
            errors.append(f'Unsquashed commit: starts with "{prefix}"')

    if lower == 'wip' or lower.startswith('wip ') or lower.startswith('wip:'):
        errors.append('WIP commit should not be merged')

    if len(first_line) < MIN_MESSAGE_LENGTH:
        errors.append(f'Message too short ({len(first_line)} chars, minimum {MIN_MESSAGE_LENGTH})')

    if first_line.strip() and first_line.strip().lower() in THROWAWAY_WORDS:
        errors.append(f'Single-word throwaway message: "{first_line.strip()}"')

    for kw in DEBUG_KEYWORDS:
        if kw in lower:
            errors.append(f'Debug session leftover: contains "{kw}"')

    # --- Warning checks ---

    for pattern in REVIEW_RESPONSE_PATTERNS:
        if pattern in lower:
            warnings.append('Review-response commit')
            break

    for pattern in FORMATTER_PATTERNS:
        if pattern in lower:
            warnings.append('Formatter-only commit')
            break

    if not parse_header(first_line):
        # Exempt merge commits
        for prefix in EXEMPT_PREFIXES:
            if first_line.startswith(prefix):
                break
        else:
            warnings.append(
                'Missing conventional commit format '
                '(e.g. "feat(ekf2): add something")'
            )

    return errors, warnings


def suggest_commit(message: str) -> str | None:
    """Suggest how to fix a bad commit message."""
    first_line = message.split('\n', 1)[0].strip()
    lower = first_line.lower()

    for prefix in FIXUP_PREFIXES:
        if lower.startswith(prefix):
            return 'Squash this into the commit it fixes'

    if lower == 'wip' or lower.startswith('wip ') or lower.startswith('wip:'):
        return 'Reword with a descriptive message (e.g. "feat(scope): what changed")'

    if len(first_line) < MIN_MESSAGE_LENGTH:
        return 'Reword with a descriptive message (e.g. "feat(ekf2): what changed")'

    if first_line.strip().lower() in THROWAWAY_WORDS:
        return 'Reword with a descriptive message (e.g. "fix(scope): what changed")'

    return None


def format_plain(data: list) -> tuple[bool, bool]:
    """Print plain text output. Returns (has_blocking, has_warnings)."""
    has_blocking = False
    has_warnings = False

    for commit in data:
        sha = commit.get('sha', '?')[:10]
        message = commit.get('commit', {}).get('message', '')
        first_line = message.split('\n', 1)[0].strip()

        errors, warnings = check_commit(message)

        if errors or warnings:
            print(f"\n  {sha}  {first_line}")

        for err in errors:
            print(f"    ERROR: {err}")
            has_blocking = True

        for warn in warnings:
            print(f"    WARNING: {warn}")
            has_warnings = True

    if has_blocking:
        print(
            "\n"
            "ERROR = must fix before merging (CI will block the PR)\n"
            "WARNING = advisory, not blocking, but recommended to fix\n"
            "\n"
            "See the contributing guide for details:\n"
            "  https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention\n",
        )

    elif has_warnings:
        print(
            "\n"
            "WARNING = advisory, not blocking, but recommended to fix\n"
            "\n"
            "See the contributing guide for details:\n"
            "  https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention\n",
        )

    return has_blocking, has_warnings


def format_markdown_blocking(data: list) -> str:
    """Format a blocking error markdown comment."""
    error_groups: dict[str, list[str]] = {}
    unique_commits: list[tuple[str, str, list[str], str]] = []

    for commit in data:
        sha = commit.get('sha', '?')[:10]
        message = commit.get('commit', {}).get('message', '')
        first_line = message.split('\n', 1)[0].strip()

        errors, _ = check_commit(message)
        if not errors:
            continue

        suggestion = suggest_commit(message) or ''
        unique_commits.append((sha, first_line, errors, suggestion))

        for err in errors:
            error_groups.setdefault(err, []).append(sha)

    lines = [
        "## \u274c Commit messages need attention before merging",
        "",
    ]

    has_large_group = any(len(shas) > 3 for shas in error_groups.values())

    if has_large_group:
        lines.extend([
            "**Issues found:**",
            "",
        ])
        for err_msg, shas in error_groups.items():
            if len(shas) > 3:
                lines.append(f"- **{len(shas)} commits**: {err_msg} "
                             f"(`{shas[0]}`, `{shas[1]}`, ... `{shas[-1]}`)")
            else:
                sha_list = ', '.join(f'`{s}`' for s in shas)
                lines.append(f"- {err_msg}: {sha_list}")

        distinct_messages = {msg for _, msg, _, _ in unique_commits}
        if len(distinct_messages) <= 5:
            lines.extend(["", "**Affected commits:**", ""])
            for sha, msg, errors, suggestion in unique_commits:
                safe_msg = msg.replace('|', '\\|')
                lines.append(f"- `{sha}` {safe_msg}")
    else:
        lines.extend([
            "| Commit | Message | Issue | Suggested fix |",
            "|--------|---------|-------|---------------|",
        ])
        for sha, msg, errors, suggestion in unique_commits:
            issues = '; '.join(errors)
            safe_msg = msg.replace('|', '\\|')
            lines.append(f"| `{sha}` | {safe_msg} | {issues} | {suggestion} |")

    lines.extend([
        "",
        "See [CONTRIBUTING.md](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention) "
        "for how to clean up commits.",
        "",
        "---",
        "*This comment will be automatically removed once the issues are resolved.*",
    ])

    return '\n'.join(lines)


def format_markdown_advisory(data: list) -> str:
    """Format an advisory warning markdown comment."""
    lines = [
        "## \U0001f4a1 Commit messages could be improved",
        "",
        "Not blocking, but these commit messages could use some cleanup.",
        "",
        "| Commit | Message | Suggestion |",
        "|--------|---------|------------|",
    ]

    for commit in data:
        sha = commit.get('sha', '?')[:10]
        message = commit.get('commit', {}).get('message', '')
        first_line = message.split('\n', 1)[0].strip()

        _, warnings = check_commit(message)
        if not warnings:
            continue

        suggestion = '; '.join(warnings)
        safe_msg = first_line.replace('|', '\\|')
        lines.append(f"| `{sha}` | {safe_msg} | {suggestion} |")

    lines.extend([
        "",
        "See the [commit message convention](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention) "
        "for details.",
        "",
        "---",
        "*This comment will be automatically removed once the issues are resolved.*",
    ])

    return '\n'.join(lines)


def main() -> None:
    markdown_stdout = '--markdown' in sys.argv
    markdown_file = None
    for i, a in enumerate(sys.argv):
        if a == '--markdown-file' and i + 1 < len(sys.argv):
            markdown_file = sys.argv[i + 1]
        elif a.startswith('--markdown-file='):
            markdown_file = a.split('=', 1)[1]

    try:
        data = json.load(sys.stdin)
    except json.JSONDecodeError as exc:
        print(f"Failed to parse JSON input: {exc}", file=sys.stderr)
        sys.exit(2)

    if not isinstance(data, list):
        print("Expected a JSON array of commit objects.", file=sys.stderr)
        sys.exit(2)

    # Always compute blocking/warning state
    has_blocking = False
    has_warnings = False
    for commit in data:
        message = commit.get('commit', {}).get('message', '')
        errors, warnings = check_commit(message)
        if errors:
            has_blocking = True
        if warnings:
            has_warnings = True

    # Generate markdown if needed
    md = None
    if has_blocking:
        md = format_markdown_blocking(data)
    elif has_warnings:
        md = format_markdown_advisory(data)

    if md:
        if markdown_stdout:
            print(md)
        if markdown_file:
            with open(markdown_file, 'w') as f:
                f.write(md + '\n')
    elif markdown_file:
        with open(markdown_file, 'w') as f:
            pass

    # Plain text output to stderr for CI logs (always, unless --markdown only)
    if not markdown_stdout:
        has_blocking, _ = format_plain(data)

    sys.exit(1 if has_blocking else 0)


if __name__ == '__main__':
    main()
