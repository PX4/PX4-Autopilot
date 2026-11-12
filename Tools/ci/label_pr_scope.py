#!/usr/bin/env python3
"""Apply at most one scope:* label to a PR: the scope at its core.

The scope comes from the PR title. Its conventional-commit scope is looked up
in TITLE_SCOPE_ALIASES, else resolved to every directory and file of
that name in the repository; the label is the one most files there carry. A
title naming an unlabelled area gets no label. Only when the title names
nothing (or a scope in NO_INFO_TITLE_SCOPES) does the PR's diff decide: the
label owning a majority of changed lines, docs counting only in docs-only PRs.

Every file belongs to at most one scope, the first in SCOPES with a matching
glob.

Scope labels added outside this workflow are left alone, and while one is present this
script applies none. Scope labels this workflow added earlier are removed when
they no longer match, so the label follows title edits and new pushes.

Usage:
    label_pr_scope.py --pr 123 [--dry-run]

Requires GITHUB_TOKEN or GH_TOKEN, and GITHUB_REPOSITORY (default PX4/PX4-Autopilot).
"""

import argparse
import os
import re
import sys
import urllib.parse

from _github_helpers import GitHubClient, fail
from conventional_commits import HEADER_PATTERN


SCOPES = (
    ('scope:release', (
        'docs/**/releases/**',
        'docs/**/release_process.md',
        'Tools/packaging/**',
        'platforms/**/package*.sh',
    )),
    ('scope:dependencies', (
        '.gitmodules',
        'package.xml',
        'src/modules/mavlink/mavlink/**',
        'src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client/**',
        'src/lib/crypto/monocypher/**',
        'src/lib/heatshrink/heatshrink/**',
        'platforms/nuttx/NuttX/**',
    )),
    ('scope:docs', (
        'docs/**',
        '.github/instructions/docs*.md',
    )),
    ('scope:infrastructure', (
        '.devcontainer/**',
        '.github/**',
        '.vscode/**',
        '.clang-tidy',
        '.dockerignore',
        'Jenkinsfile',
        'Tools/ci/**',
        'Tools/docker/**',
    )),
    ('scope:boards', (
        'boards/**',
        'src/drivers/px4io/**',
        'src/modules/px4iofirmware/**',
    )),
    ('scope:build-system', (
        'CMakeLists.txt',
        'Makefile',
        'Kconfig',
        'cmake/**',
        'platforms/*/cmake/**',
        'platforms/*/Kconfig',
    )),
    ('scope:parameters', (
        'src/lib/parameters/**',
        'src/modules/param/**',
        'Tools/param_metadata/**',
    )),
    ('scope:commander', (
        'src/modules/commander/**',
    )),
    ('scope:control', (
        'src/modules/*_att_control/**',
        'src/modules/*_rate_control/**',
        'src/modules/*_pos_control/**',
        'src/modules/*_mode_manager/**',
        'src/modules/control_allocator/**',
        'src/modules/flight_mode_manager/**',
        'src/modules/fw_lateral_longitudinal_control/**',
        'src/modules/mc_hover_thrust_estimator/**',
        'src/modules/mc_nn_control/**',
        'src/modules/mc_raptor/**',
        'src/modules/rover_*/**',
        'src/drivers/actuators/**',
        'src/drivers/dshot/**',
        'src/drivers/pwm_out/**',
        'src/drivers/pca9685_pwm_out/**',
        'src/drivers/tap_esc/**',
        'src/lib/control_allocation/**',
        'src/lib/npfg/**',
        'src/lib/rate_control/**',
        'src/lib/tecs/**',
    )),
    ('scope:estimation', (
        'src/modules/ekf2/**',
        'src/modules/local_position_estimator/**',
        'src/modules/attitude_estimator_q/**',
        'src/modules/landing_target_estimator/**',
        'src/modules/mag_bias_estimator/**',
        'src/modules/gyro_calibration/**',
        'src/modules/gyro_fft/**',
        'Tools/ecl_ekf/**',
    )),
    ('scope:mavlink', (
        'src/modules/mavlink/**',
        'src/drivers/telemetry/**',
        'src/drivers/transponder/**',
        'Tools/HIL/**',
    )),
    ('scope:offboard', (
        'src/modules/uxrce_dds_client/**',
        'src/modules/zenoh/**',
        'msg/OffboardControlMode.msg',
        'msg/TrajectorySetpoint.msg',
        'msg/versioned/OffboardControlMode.msg',
        'msg/versioned/TrajectorySetpoint.msg',
    )),
    ('scope:navigation', (
        'src/modules/navigator/**',
        'src/modules/dataman/**',
        'src/modules/land_detector/**',
        'src/modules/payload_deliverer/**',
        'src/lib/collision_prevention/**',
        'src/lib/geofence/**',
        'src/lib/landing_slope/**',
        'src/lib/rtl/**',
        'src/lib/takeoff/**',
        'src/lib/weather_vane/**',
    )),
    ('scope:simulation', (
        'launch/**',
        'posix-configs/**',
        'src/modules/simulation/**',
        'Tools/simulation/**',
        'ROMFS/px4fmu_common/init.d-posix/**',
        'platforms/posix/**',
    )),
    ('scope:sensors', (
        'src/modules/sensors/**',
        'src/modules/airspeed_selector/**',
        'src/modules/battery_status/**',
        'src/modules/esc_battery/**',
        'src/modules/temperature_compensation/**',
        'src/drivers/adc/**',
        'src/drivers/barometer/**',
        'src/drivers/batt_smbus/**',
        'src/drivers/differential_pressure/**',
        'src/drivers/distance_sensor/**',
        'src/drivers/gnss/**',
        'src/drivers/gps/**',
        'src/drivers/hygrometer/**',
        'src/drivers/imu/**',
        'src/drivers/ins/**',
        'src/drivers/irlock/**',
        'src/drivers/magnetometer/**',
        'src/drivers/optical_flow/**',
        'src/drivers/power_monitor/**',
        'src/drivers/pps_capture/**',
        'src/drivers/rpm/**',
        'src/drivers/rpm_capture/**',
        'src/drivers/smart_battery/**',
        'src/drivers/tattu_can/**',
        'src/drivers/temperature_sensor/**',
        'src/drivers/uwb/**',
        'src/drivers/wind_sensor/**',
    )),
    ('scope:drivers', (
        'src/drivers/**',
    )),
    ('scope:tools', (
        'Tools/**',
        'msg/tools/**',
        'src/templates/**',
    )),
)

# Title scopes that name an area rather than a path, matched case-insensitively
# on the part before any '/' ("boards/agam" -> "boards").
TITLE_SCOPE_ALIASES = {
    'boards': 'scope:boards',
    'build': 'scope:build-system',
    'cmake': 'scope:build-system',
    'kconfig': 'scope:build-system',
    'ci': 'scope:infrastructure',
    'docs': 'scope:docs',
    'gnss': 'scope:sensors',
    'i18n': 'scope:docs',
    'param': 'scope:parameters',
    'params': 'scope:parameters',
    'parameters': 'scope:parameters',
    'release': 'scope:release',
    'releases': 'scope:release',
    'rover': 'scope:control',
    'sih': 'scope:simulation',
    'sim': 'scope:simulation',
    'simulation': 'scope:simulation',
    'sitl': 'scope:simulation',
}

# Conventional-commit types that fix the scope whatever the title scope says
# ("ci(macos)" is CI work, not macOS tooling).
TYPE_SCOPES = {
    'ci': 'scope:infrastructure',
}

# Title scopes naming directories found all over the tree, which say nothing
# about the area.
NO_INFO_TITLE_SCOPES = {'msg', 'src', 'test', 'tests'}

MAJORITY = 0.5


def _glob_to_regex(glob):
    out = ''
    i = 0
    while i < len(glob):
        if glob.startswith('**/', i):
            out += '(?:.*/)?'
            i += 3
        elif glob.startswith('**', i):
            out += '.*'
            i += 2
        elif glob[i] == '*':
            out += '[^/]*'
            i += 1
        else:
            out += re.escape(glob[i])
            i += 1
    return re.compile(out + '$')


_SCOPE_PATTERNS = [(label, [_glob_to_regex(g) for g in globs]) for label, globs in SCOPES]


def scope_of(path):
    for label, patterns in _SCOPE_PATTERNS:
        if any(p.match(path) for p in patterns):
            return label
    return None


def _majority(weights, total):
    if not weights:
        return None
    label = max(weights, key=weights.get)
    return label if weights[label] / total >= MAJORITY else None


def _names(path, name):
    if '/' in name:
        return ('/' + path + '/').find('/' + name + '/') >= 0
    return name in path.split('/')


def title_area_scope(title_scope, tree):
    """Label for the area a title scope names, (True, label-or-None), or (False, None) if it names nothing."""
    name = title_scope.lower().strip('/')
    area = [p for p in tree if not p.startswith('docs/') and _names(p.lower(), name)]
    if not area:
        return False, None
    weights = {}
    for path in area:
        label = scope_of(path)
        if label:
            weights[label] = weights.get(label, 0) + 1
    return True, _majority(weights, len(area))


def diff_scope(files):
    """Label owning a majority of the PR's changed lines. `files` are GitHub PR file dicts."""
    # Docs lines are cheap to write in volume, so they only decide docs-only PRs.
    code = [f for f in files if not f['filename'].startswith('docs/')]
    weights = {}
    total = 0
    for f in code or files:
        # Renames and binary files report zero changed lines but still count.
        lines = max(1, f['additions'] + f['deletions'])
        total += lines
        label = scope_of(f['filename'])
        if label:
            weights[label] = weights.get(label, 0) + lines
    return _majority(weights, total)


def pick_scope(title, files, tree):
    """Scope label for a PR, or None. `tree` lists every path in the base branch."""
    match = HEADER_PATTERN.match(title)
    if match:
        commit_type, title_scope = match.group(1), match.group(2)
        if commit_type in TYPE_SCOPES:
            return TYPE_SCOPES[commit_type]
        alias = TITLE_SCOPE_ALIASES.get(title_scope.split('/')[0].lower())
        if alias:
            return alias
        if title_scope.lower() not in NO_INFO_TITLE_SCOPES:
            found, label = title_area_scope(title_scope, tree)
            if found:
                return label
    return diff_scope(files)


def _added_by_workflow(client, repo, pr):
    """Labels whose most recent 'labeled' event came from GitHub Actions."""
    actor = {}
    for event in client.paginated(f'repos/{repo}/issues/{pr}/events'):
        if event.get('event') == 'labeled':
            actor[event['label']['name']] = (event.get('actor') or {}).get('login')
    return {name for name, login in actor.items() if login == 'github-actions[bot]'}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--pr', type=int, required=True)
    parser.add_argument('--dry-run', action='store_true', help='print the decision, change nothing')
    args = parser.parse_args()

    token = os.environ.get('GITHUB_TOKEN') or os.environ.get('GH_TOKEN')
    if not token:
        fail('GITHUB_TOKEN or GH_TOKEN is required')
    repo = os.environ.get('GITHUB_REPOSITORY', 'PX4/PX4-Autopilot')
    client = GitHubClient(token, user_agent='px4-label-pr-scope')

    pr, _ = client.request('GET', f'repos/{repo}/pulls/{args.pr}')
    files = list(client.paginated(f'repos/{repo}/pulls/{args.pr}/files'))
    tree, _ = client.request('GET', f'repos/{repo}/git/trees/{pr["base"]["sha"]}?recursive=1')
    paths = [entry['path'] for entry in tree['tree'] if entry['type'] != 'tree']

    current = {label['name'] for label in pr['labels'] if label['name'].startswith('scope:')}
    ours = current & _added_by_workflow(client, repo, args.pr)
    theirs = current - ours

    target = None if theirs else pick_scope(pr['title'], files, paths)
    remove = sorted(ours - {target})
    add = target if target and target not in current else None

    print(f'#{args.pr} {pr["title"]!r}: {target or "no scope"}'
          + (f' (kept {", ".join(sorted(theirs))})' if theirs else ''))
    if args.dry_run:
        print(f'would add {add}, remove {remove}')
        return

    for label in remove:
        print(f'removing {label}')
        client.request('DELETE', f'repos/{repo}/issues/{args.pr}/labels/{urllib.parse.quote(label, safe="")}')
    if add:
        print(f'adding {add}')
        client.request('POST', f'repos/{repo}/issues/{args.pr}/labels', {'labels': [add]})


if __name__ == '__main__':
    sys.exit(main())
