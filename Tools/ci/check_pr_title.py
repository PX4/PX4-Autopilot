#!/usr/bin/env python3
"""Validate that a PR title follows conventional commits format.

Format: type(scope): description

Can output plain text for CI logs or markdown for PR comments.
"""

import re
import sys

from conventional_commits import (
    CONVENTIONAL_TYPES,
    EXEMPT_PREFIXES,
    parse_header,
    suggest_scope,
    suggest_type,
)


def suggest_title(title: str) -> str | None:
    """Try to suggest a corrected title in conventional commits format."""
    stripped = title.strip()

    # Remove common bracket prefixes like [docs], [CI], etc.
    bracket_match = re.match(r'^\[([^\]]+)\]\s*(.+)', stripped)
    if bracket_match:
        prefix = bracket_match.group(1).strip().lower()
        rest = bracket_match.group(2).strip()
        rest = re.sub(r'^[\-:]\s*', '', rest).strip()
        if len(rest) >= 5:
            # Try to map bracket content to a type
            commit_type = prefix if prefix in CONVENTIONAL_TYPES else suggest_type(rest)
            scope = suggest_scope(rest)
            if scope:
                return f"{commit_type}({scope}): {rest}"

    # Already has old-style "subsystem: description" format - convert it
    colon_match = re.match(r'^([a-zA-Z][a-zA-Z0-9_/\-\. ]*): (.+)$', stripped)
    if colon_match:
        old_subsystem = colon_match.group(1).strip()
        desc = colon_match.group(2).strip()
        if len(desc) >= 5:
            commit_type = suggest_type(desc)
            # Use the old subsystem as scope (clean it up)
            scope = old_subsystem.lower().replace(' ', '_')
            return f"{commit_type}({scope}): {desc}"

    # No format at all - try to guess both type and scope
    commit_type = suggest_type(stripped)
    scope = suggest_scope(stripped)
    if scope:
        desc = stripped[0].lower() + stripped[1:] if stripped else stripped
        return f"{commit_type}({scope}): {desc}"

    return None


def check_title(title: str) -> bool:
    title = title.strip()

    if not title:
        print("PR title is empty.", file=sys.stderr)
        return False

    for prefix in EXEMPT_PREFIXES:
        if title.startswith(prefix):
            return True

    if parse_header(title):
        return True

    types_str = ', '.join(f'`{t}`' for t in CONVENTIONAL_TYPES.keys())
    print(
        f"PR title does not match conventional commits format.\n"
        f"\n"
        f"  Title: {title}\n"
        f"\n"
        f"Expected format:  type(scope): description\n"
        f"\n"
        f"Valid types: {types_str}\n"
        f"\n"
        f"Good examples:\n"
        f"  feat(ekf2): add height fusion timeout\n"
        f"  fix(mavlink): correct BATTERY_STATUS_V2 parsing\n"
        f"  ci(workflows): migrate to reusable workflows\n"
        f"  feat(boards/px4_fmu-v6x)!: remove deprecated driver API\n"
        f"\n"
        f"Bad examples:\n"
        f"  fix stuff\n"
        f"  Update file\n"
        f"  ekf2: fix something   (missing type prefix)\n"
        f"\n"
        f"See the contributing guide for details:\n"
        f"  https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention\n",
        file=sys.stderr,
    )
    return False


def format_markdown(title: str) -> str:
    """Format a markdown PR comment body for a bad title."""
    lines = [
        "## \u274c PR title needs conventional commit format",
        "",
        "Expected format: `type(scope): description` "
        "([conventional commits](https://www.conventionalcommits.org/)).",
        "",
        "**Your title:**",
        f"> {title}",
        "",
    ]

    suggestion = suggest_title(title)
    if suggestion:
        lines.extend([
            "**Suggested fix:**",
            f"> {suggestion}",
            "",
        ])

    lines.extend([
        "**To fix this:** click the ✏️ next to the PR title at the top "
        "of this page and update it.",
        "",
        "See [CONTRIBUTING.md](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention) "
        "for details.",
        "",
        "---",
        "*This comment will be automatically removed once the issue is resolved.*",
    ])

    return '\n'.join(lines)


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description='Check PR title format')
    parser.add_argument('title', help='The PR title to validate')
    parser.add_argument('--markdown', action='store_true',
                        help='Output markdown to stdout on failure')
    parser.add_argument('--markdown-file', metavar='FILE',
                        help='Write markdown to FILE on failure')
    args = parser.parse_args()

    passed = check_title(args.title)

    if not passed:
        md = format_markdown(args.title)
        if args.markdown:
            print(md)
        if args.markdown_file:
            with open(args.markdown_file, 'w') as f:
                f.write(md + '\n')
    elif args.markdown_file:
        with open(args.markdown_file, 'w') as f:
            pass

    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
