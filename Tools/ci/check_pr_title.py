#!/usr/bin/env python3
"""Validate that a PR title follows PX4's 'subsystem: description' convention.

Can output plain text for CI logs or markdown for PR comments.
"""

import re
import sys

# subsystem prefix, colon, space, then at least 5 chars of description
TITLE_PATTERN = re.compile(r'^[a-zA-Z][a-zA-Z0-9_/\-\. ]*: .{5,}')

# Titles that are exempt from the subsystem pattern
EXEMPT_PREFIXES = ('Revert "', 'Merge ')

# Common path-based subsystem guesses from title keywords
KEYWORD_SUBSYSTEMS = [
    (r'\b(ekf|estimator|height|fusion|imu|baro|gps|mag)\b', 'ekf2'),
    (r'\b(mavlink|MAVLink|MAVLINK|command_int|heartbeat)\b', 'mavlink'),
    (r'\b(uorb|orb|pub|sub|topic)\b', 'uORB'),
    (r'\b(board|fmu|nuttx|stm32|driver)\b', 'boards'),
    (r'\b(mixer|actuator|motor|servo|pwm|dshot)\b', 'actuators'),
    (r'\b(battery|power)\b', 'battery'),
    (r'\b(param|parameter)\b', 'param'),
    (r'\b(log|logger|sdlog)\b', 'logger'),
    (r'\b(sensor|accel|gyro)\b', 'sensors'),
    (r'\b(land|takeoff|rtl|mission|navigator|geofence)\b', 'navigator'),
    (r'\b(position|velocity|attitude|rate)\s*(control|ctrl)\b', 'control'),
    (r'\b(mc|multicopter|quad)\b', 'multicopter'),
    (r'\b(fw|fixedwing|fixed.wing|plane)\b', 'fixedwing'),
    (r'\b(vtol|transition)\b', 'vtol'),
    (r'\b(ci|workflow|github.action|pipeline|build)\b', 'CI'),
    (r'\b(doc|docs|documentation|readme)\b', 'docs'),
    (r'\b(cmake|make|toolchain|compiler)\b', 'build'),
    (r'\b(sitl|simulation|gazebo|jmavsim|sih)\b', 'simulation'),
    (r'\b(can|uavcan|cyphal|dronecan)\b', 'CAN'),
    (r'\b(serial|uart|spi|i2c)\b', 'drivers'),
    (r'\b(safety|failsafe|arm|disarm|kill)\b', 'safety'),
    (r'\b(tune|buzzer|led|tone)\b', 'tunes'),
    (r'\b(rc|radio|sbus|crsf|elrs|dsm)\b', 'RC'),
    (r'\b(gps|gnss|rtk|ubx)\b', 'gps'),
    (r'\b(optical.flow|flow|rangefinder|lidar|distance)\b', 'sensors'),
    (r'\b(orbit|follow|offboard)\b', 'flight_mode'),
]


def suggest_title(title: str) -> str | None:
    """Try to suggest a corrected title with a subsystem prefix."""
    stripped = title.strip()

    # Remove common bracket prefixes like [docs], [CI], etc.
    bracket_match = re.match(r'^\[([^\]]+)\]\s*(.+)', stripped)
    if bracket_match:
        prefix = bracket_match.group(1).strip()
        rest = bracket_match.group(2).strip()
        # Clean up the rest: remove leading separators
        rest = re.sub(r'^[\-:]\s*', '', rest).strip()
        if len(rest) >= 5:
            return f"{prefix}: {rest}"

    # Has a colon but description too short?
    colon_match = re.match(r'^([a-zA-Z][a-zA-Z0-9_/\-\. ]*): (.*)$', stripped)
    if colon_match:
        prefix = colon_match.group(1)
        desc = colon_match.group(2).strip()
        if len(desc) < 5:
            return None  # Can't fix a too-short description
        return f"{prefix}: {desc}"

    # Try to guess subsystem from keywords
    lower = stripped.lower()
    for pattern, subsystem in KEYWORD_SUBSYSTEMS:
        if re.search(pattern, lower, re.IGNORECASE):
            # Lowercase the first char of the original title for the description
            desc = stripped[0].lower() + stripped[1:] if stripped else stripped
            return f"{subsystem}: {desc}"

    return None


def check_title(title: str) -> bool:
    title = title.strip()

    if not title:
        print("PR title is empty.", file=sys.stderr)
        return False

    for prefix in EXEMPT_PREFIXES:
        if title.startswith(prefix):
            return True

    if TITLE_PATTERN.match(title):
        return True

    print(
        f"PR title does not match the expected format.\n"
        f"\n"
        f"  Title: {title}\n"
        f"\n"
        f"Expected pattern:  subsystem: short description (5+ chars)\n"
        f"\n"
        f"Good examples:\n"
        f'  ekf2: fix height fusion timeout\n'
        f'  mavlink: add BATTERY_STATUS_V2 support\n'
        f'  boards/px4_fmu-v6x: enable UAVCAN\n'
        f'  CI: migrate to reusable workflows\n'
        f"\n"
        f"Bad examples:\n"
        f'  fix stuff\n'
        f'  Update file\n'
        f'  changes\n'
        f"\n"
        f"See the contributing guide for details:\n"
        f"  https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention\n",
        file=sys.stderr,
    )
    return False


def format_markdown(title: str) -> str:
    """Format a markdown PR comment body for a bad title."""
    lines = [
        "## PR Title",
        "",
        "Your PR title doesn't follow the required `subsystem: description` format.",
        "",
        "**Format:** `subsystem: short description of the change`",
        "",
        "The subsystem prefix identifies which part of PX4 is affected. "
        "The description (at least 5 characters) should clearly explain "
        "what the change does.",
        "",
        "| Part | Rule |",
        "|------|------|",
        "| `subsystem` | module, board, or area name (e.g. `ekf2`, `mavlink`, `boards/px4_fmu-v6x`, `CI`) |",
        "| `:` + space | required separator |",
        "| `description` | what the change does, at least 5 characters |",
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
        "**To fix this:** click the **pencil icon** (:pencil2:) next to the PR title at the top "
        "of this page, edit it to match the format, and press Enter.",
        "",
        "<details>",
        "<summary>Examples of good PR titles</summary>",
        "",
        "```",
        "ekf2: fix height fusion timeout",
        "mavlink: add BATTERY_STATUS_V2 support",
        "boards/px4_fmu-v6x: enable UAVCAN",
        "CI: migrate to reusable workflows",
        "docs: update EKF tuning guide",
        "navigator: fix RTL altitude calculation",
        "multicopter: improve yaw rate setpoint",
        "```",
        "",
        "</details>",
        "",
        "<details>",
        "<summary>PX4 commit message convention</summary>",
        "",
        "PX4 uses the `subsystem: description` format for both PR titles and commit messages.",
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
        "</details>",
        "",
        "See the full [commit message convention](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md#commit-message-convention) "
        "in the contributing guide.",
        "",
        "---",
        "*This comment will be automatically removed once the issue is resolved.*",
    ])

    return '\n'.join(lines)


def main() -> None:
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print(f"Usage: {sys.argv[0]} <pr-title> [--markdown]", file=sys.stderr)
        sys.exit(2)

    title = sys.argv[1]
    markdown = '--markdown' in sys.argv

    passed = check_title(title)

    if markdown:
        if not passed:
            print(format_markdown(title))
        # On pass, output nothing (no comment needed)

    if passed:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
