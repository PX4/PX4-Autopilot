#!/usr/bin/env python3
"""Validate commit messages in a PR against PX4 conventions.

Reads a JSON array of GitHub commit objects from stdin (as returned by the
GitHub API's /pulls/{n}/commits endpoint) and checks each message for
blocking errors and advisory warnings.

With --markdown, outputs a formatted PR comment body instead of plain text.
"""

import json
import re
import sys

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

# Advisory: subsystem prefix pattern
SUBSYSTEM_PATTERN = re.compile(r'^[a-zA-Z][a-zA-Z0-9_/\-\. ]*: ')

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
            warnings.append('Review-response commit: consider squashing before merge')
            break

    for pattern in FORMATTER_PATTERNS:
        if pattern in lower:
            warnings.append('Formatter-only commit: consider squashing into parent commit')
            break

    if not SUBSYSTEM_PATTERN.match(first_line):
        # Exempt merge and revert commits
        if not first_line.startswith('Revert "') and not first_line.startswith('Merge '):
            warnings.append('Missing subsystem prefix (e.g. "ekf2: fix something")')

    return errors, warnings


def suggest_commit(message: str) -> str | None:
    """Suggest how to fix a bad commit message."""
    first_line = message.split('\n', 1)[0].strip()
    lower = first_line.lower()

    # fixup/squash/amend -> suggest squashing
    for prefix in FIXUP_PREFIXES:
        if lower.startswith(prefix):
            return 'Squash this into the commit it fixes'

    # WIP
    if lower == 'wip' or lower.startswith('wip ') or lower.startswith('wip:'):
        return 'Squash into parent or reword with a descriptive message'

    # Too short or throwaway
    if len(first_line) < MIN_MESSAGE_LENGTH:
        return 'Reword with a descriptive message (e.g. "subsystem: what changed")'

    if first_line.strip().lower() in THROWAWAY_WORDS:
        return 'Reword with a descriptive message (e.g. "subsystem: what changed")'

    return None


def format_plain(data: list) -> tuple[bool, bool]:
    """Print plain text output. Returns (has_blocking, has_warnings)."""
    has_blocking = False
    has_warnings = False
    total_commits = len(data)

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
            "Commit message errors must be fixed before merging.\n"
            "\n"
            "Please squash or reword your commits before this PR can be merged.\n"
            "Each commit on main should have a clear, descriptive message like:\n"
            "\n"
            "  ekf2: fix height fusion timeout\n"
            "  mavlink: add BATTERY_STATUS_V2 support\n"
            "  boards/px4_fmu-v6x: enable UAVCAN\n"
            "\n"
            f"To squash everything into one commit:\n"
            f"  git rebase -i HEAD~{total_commits}\n"
            "  git push --force-with-lease  # safe for PR branches\n"
            "\n"
            "See the contributing guide for details:\n"
            "  https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention\n",
        )

    elif has_warnings:
        print(
            "\n"
            "WARNING = advisory, not blocking, but recommended to fix\n"
            "\n"
            "Warnings above are advisory. Consider squashing cleanup commits\n"
            "(review responses, formatting fixes) into their parent commits\n"
            "before merge so that main stays clean.\n"
            "\n"
            "See the contributing guide for details:\n"
            "  https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention\n",
        )

    return has_blocking, has_warnings


def format_markdown_blocking(data: list) -> str:
    """Format a blocking error markdown comment."""
    total_commits = len(data)

    # Group commits by error category to avoid repeating identical rows
    error_groups: dict[str, list[str]] = {}  # error_msg -> [sha, ...]
    unique_commits: list[tuple[str, str, list[str], str]] = []  # (sha, msg, errors, suggestion)

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
        "## Commit Messages",
        "",
        "Some commits in this PR have messages that **must be fixed before merging**.",
        "",
        "Every commit that lands on `main` becomes permanent project history. "
        "Each commit message should use the `subsystem: description` format and "
        "clearly describe what changed.",
        "",
    ]

    # If many commits share the same error, show a grouped summary
    # Otherwise show individual rows
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

        # Show a few example commits if not all identical
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
        "**How to fix:**",
        "",
        "\"Squashing\" means combining multiple commits into one. "
        "If you have a single meaningful commit buried under cleanup or review commits, "
        "squash them all into one:",
        "```bash",
        f"git rebase -i HEAD~{total_commits}",
        "```",
        "",
        "This opens an editor that looks like this:",
        "```",
    ])

    # Show a realistic rebase editor example using actual commits
    example_commits = []
    for commit in data[:4]:
        sha = commit.get('sha', '?')[:7]
        msg = commit.get('commit', {}).get('message', '').split('\n', 1)[0].strip()[:50]
        example_commits.append((sha, msg))

    if example_commits:
        lines.append(f"pick {example_commits[0][0]} {example_commits[0][1]}")
        for sha, msg in example_commits[1:]:
            lines.append(f"pick {sha} {msg}")
        if total_commits > 4:
            lines.append(f"# ... and {total_commits - 4} more commits")

    lines.extend([
        "```",
        "",
        "Change `pick` to `squash` (or `s`) for all commits except the first, "
        "then save and close. Git will let you write a new combined message.",
        "",
        "If each commit is a separate logical change, reword the bad ones instead:",
        "```bash",
        f"git rebase -i HEAD~{total_commits}",
        "# change 'pick' to 'reword' (or 'r') for the commits to fix",
        "# git will prompt you to write a new message for each one",
        "```",
        "",
        "Then push your changes:",
        "```bash",
        "git push --force-with-lease",
        "```",
        "(`--force-with-lease` is safe on PR branches. It only updates the remote "
        "if nobody else has pushed since your last fetch.)",
        "",
    ])

    lines.extend(_convention_section())

    lines.extend([
        "",
        "See the full [commit message convention](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention) "
        "in the contributing guide.",
        "",
        "---",
        "*This comment will be automatically removed once the issues are resolved.*",
    ])

    return '\n'.join(lines)


def _convention_section() -> list[str]:
    """Shared PX4 commit convention details for markdown comments."""
    return [
        "<details>",
        "<summary>PX4 commit message convention</summary>",
        "",
        "PX4 uses the `subsystem: description` format for all commit messages. "
        "This keeps `git log` and `git blame` readable and makes it easy to "
        "generate changelogs.",
        "",
        "The **subsystem** is the module, driver, board, or area of PX4 that the change affects. "
        "Common subsystems include: `ekf2`, `mavlink`, `navigator`, `sensors`, `drivers`, "
        "`boards/px4_fmu-v6x`, `CI`, `docs`, `simulation`, `multicopter`, `fixedwing`, `vtol`.",
        "",
        "**How to find the right subsystem:** look at the directory path of the files you changed. "
        "For example, changes in `src/modules/ekf2/` use `ekf2`, changes in `src/drivers/imu/` "
        "use `drivers/imu`, and changes in `.github/workflows/` use `CI`.",
        "",
        "The **description** should be a short, imperative summary of the change (e.g. "
        '"fix timeout", "add support for X", "remove deprecated API").',
        "",
        "Good commit messages:",
        "```",
        "ekf2: fix height fusion timeout",
        "mavlink: add BATTERY_STATUS_V2 support",
        "boards/px4_fmu-v6x: enable UAVCAN",
        "```",
        "",
        "Commits to avoid (squash these before merging):",
        "```",
        "fix                          # too vague",
        "apply suggestions from code review  # squash into parent",
        "do make format               # squash into parent",
        "WIP: trying something        # not ready for main",
        "```",
        "",
        "</details>",
    ]


def format_markdown_advisory(data: list) -> str:
    """Format an advisory warning markdown comment."""
    total_commits = len(data)
    lines = [
        "## Commit Messages (advisory)",
        "",
        "This is **not blocking**, but we noticed some commit messages that could be improved.",
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
        "**Why this matters:** every commit on `main` is permanent project history. "
        "Clean, prefixed messages (`subsystem: description`) make `git log`, "
        "`git blame`, and release notes much more useful.",
        "",
        "Consider squashing (combining) review-response commits "
        "(\"address review\", \"apply suggestions\") "
        "and formatting commits (\"make format\") into their parent commit before merge:",
        "```bash",
        f"git rebase -i HEAD~{total_commits}",
        "# change 'pick' to 'squash' (or 's') for the commits to combine",
        "git push --force-with-lease  # safe for PR branches",
        "```",
        "",
    ])

    lines.extend(_convention_section())

    lines.extend([
        "",
        "See the full [commit message convention](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention) "
        "in the contributing guide.",
        "",
        "---",
        "*This comment will be automatically removed once the issues are resolved.*",
    ])

    return '\n'.join(lines)


def main() -> None:
    markdown = '--markdown' in sys.argv

    try:
        data = json.load(sys.stdin)
    except json.JSONDecodeError as exc:
        print(f"Failed to parse JSON input: {exc}", file=sys.stderr)
        sys.exit(2)

    if not isinstance(data, list):
        print("Expected a JSON array of commit objects.", file=sys.stderr)
        sys.exit(2)

    if markdown:
        # Check all commits to determine result type
        has_blocking = False
        has_warnings = False
        for commit in data:
            message = commit.get('commit', {}).get('message', '')
            errors, warnings = check_commit(message)
            if errors:
                has_blocking = True
            if warnings:
                has_warnings = True

        if has_blocking:
            print(format_markdown_blocking(data))
            sys.exit(1)
        elif has_warnings:
            print(format_markdown_advisory(data))
            sys.exit(0)
        # All clean: no output, exit 0
        sys.exit(0)
    else:
        has_blocking, _ = format_plain(data)
        sys.exit(1 if has_blocking else 0)


if __name__ == '__main__':
    main()
