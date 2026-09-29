"""Firmware gate: know exactly which firmware is on the board under test.

A qualification run against an unknown build is worthless. These helpers
let the suite establish the board's identity (`ver all` over the nsh
shell), parse and validate .px4 firmware files, flash via
Tools/px4_uploader.py, build a target from this source tree, download a
released artifact from GitHub, and verify the flashed identity against the
expectation.

Verified facts this module relies on:
- .px4 files are JSON with board_id / version / git_identity written by
  Tools/px_mkfw.py (lines 51-101); recent builds also carry the full
  git_hash. Tools/px4_uploader.py requires image / board_id / image_size /
  image_maxsize (line 354) and compares board_id against the bootloader's
  board_type.
- `ver all` prints 'PX4 git-hash:', 'PX4 version:', 'HW arch:', and
  'OS version:' lines (src/systemcmds/ver/ver.cpp:164-250).
- HW arch is the build target uppercased with '-' replaced by '_'
  (cmake/px4_config.cmake:108-109), so the reverse mapping is resolved by
  enumerating boards/<vendor>/<model>.
- boards/<vendor>/<model>/firmware.prototype carries the numeric board_id,
  enabling a wrong-board check before flashing.
- Release assets are named <vendor>_<model>_default.px4 (verified against
  the v1.17.0 release listing).
"""

import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time

from . import (DEFAULT_BAUD, SHELL_OPEN_TIMEOUT, MavlinkShell,
               is_serial_device, wait_reconnect)

GITHUB_REPO = 'PX4/PX4-Autopilot'
FLASH_TIMEOUT_S = 600.0
BUILD_TIMEOUT_S = 1800.0
DOWNLOAD_TIMEOUT_S = 300.0
REBOOT_HINT_AFTER_ATTEMPTS = 3


def repo_root():
    """PX4-Autopilot checkout root, derived from this file's location."""
    return os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))


def parse_px4_metadata(path):
    """Parse a .px4 firmware file (JSON) and validate its metadata.

    Returns a dict with git_identity, git_hash (may be None on old files),
    board_id, version, summary, description, image_size, image_maxsize.
    Raises ValueError with a clear message if the file does not parse or
    required fields are missing.
    """
    try:
        with open(path) as f:
            desc = json.load(f)
    except OSError as e:
        raise ValueError('cannot read {}: {}'.format(path, e))
    except ValueError as e:
        raise ValueError('{} is not a valid .px4 file (not JSON): {}'.format(path, e))
    if not isinstance(desc, dict):
        raise ValueError('{} is not a valid .px4 file (not a JSON object)'.format(path))

    missing = [k for k in ('git_identity', 'board_id', 'version', 'image') if k not in desc]
    if missing:
        raise ValueError('{} is missing .px4 metadata fields: {}'.format(
            path, ', '.join(missing)))

    return {k: desc.get(k) for k in (
        'git_identity', 'git_hash', 'board_id', 'version',
        'summary', 'description', 'image_size', 'image_maxsize')}


def expected_hash_from_metadata(meta):
    """Best hash expectation from .px4 metadata.

    Prefer the full git_hash; fall back to the short hash after '-g' in the
    git_identity describe string (e.g. v1.18.0-alpha1-592-g0c000d596a4).
    """
    if meta.get('git_hash'):
        return str(meta['git_hash'])
    m = re.search(r'-g([0-9a-fA-F]+)$', str(meta.get('git_identity') or ''))
    if m:
        return m.group(1)
    return str(meta.get('git_identity') or '')


def hashes_match(board_hash, expected_prefix):
    """Prefix comparison in either direction (metadata hashes are short)."""
    a = str(board_hash).strip().lower()
    b = str(expected_prefix).strip().lower()
    if not a or not b:
        return False
    return a.startswith(b) or b.startswith(a)


def board_identity(mav, timeout=15):
    """Read the board's identity via `ver all` over the nsh shell.

    Returns (identity_dict, None) or (None, error_string). The dict has
    git_hash, version, hw_arch, os_version (whichever parsed) plus the raw
    output.
    """
    shell = MavlinkShell(mav)
    if not shell.open():
        return None, 'nsh shell did not respond within {:.0f}s'.format(SHELL_OPEN_TIMEOUT)
    try:
        out, timed_out = shell.run('ver all', timeout=timeout)
    finally:
        shell.close()
    if timed_out:
        return None, "'ver all' stalled (no completion within {}s)".format(timeout)

    ident = {'raw': out}
    prefixes = (('PX4 git-hash:', 'git_hash'),
                ('PX4 version:', 'version'),
                ('HW arch:', 'hw_arch'),
                ('OS version:', 'os_version'))
    for line in out.splitlines():
        stripped = line.strip()
        for prefix, key in prefixes:
            if stripped.startswith(prefix) and key not in ident:
                ident[key] = stripped[len(prefix):].strip()
    if 'git_hash' not in ident:
        return None, "could not parse 'PX4 git-hash:' from ver all output " \
                     '(got {} bytes)'.format(len(out))
    return ident, None


def format_identity(ident):
    """One-line-per-field operator-facing summary of a board identity."""
    lines = []
    for key, label in (('git_hash', 'PX4 git-hash'), ('version', 'PX4 version'),
                       ('hw_arch', 'HW arch'), ('os_version', 'OS version')):
        if ident.get(key):
            lines.append('  {:<14} {}'.format(label + ':', ident[key]))
    return '\n'.join(lines)


def stamp_identity(report_dir, identity, extra=None):
    """Write firmware.json into a report dir so every result is traceable."""
    data = {k: v for k, v in identity.items() if k != 'raw'}
    if extra:
        data.update(extra)
    path = os.path.join(report_dir, 'firmware.json')
    try:
        with open(path, 'w') as f:
            json.dump(data, f, indent=2, sort_keys=True)
            f.write('\n')
    except OSError as e:
        print('[INFO] could not write {}: {}'.format(path, e), flush=True)
    return path


def verify_identity(report, mav, expected_hash_prefix):
    """Named check 'firmware_identity': board git hash vs the expectation.

    Returns (ok, identity_or_None).
    """
    ident, err = board_identity(mav)
    if ident is None:
        report.fail('firmware_identity', err)
        return False, None
    ok = hashes_match(ident['git_hash'], expected_hash_prefix)
    report.check('firmware_identity',
                 ok,
                 'board reports {} , expected {}'.format(
                     ident['git_hash'], expected_hash_prefix))
    return ok, ident


def list_board_targets(root=None):
    """Map build target -> board dir for every boards/<vendor>/<model>."""
    root = root or repo_root()
    targets = {}
    boards_dir = os.path.join(root, 'boards')
    if not os.path.isdir(boards_dir):
        return targets
    for vendor in sorted(os.listdir(boards_dir)):
        vdir = os.path.join(boards_dir, vendor)
        if not os.path.isdir(vdir):
            continue
        for model in sorted(os.listdir(vdir)):
            mdir = os.path.join(vdir, model)
            if os.path.isdir(mdir):
                targets['{}_{}'.format(vendor, model)] = mdir
    return targets


def infer_build_target(hw_arch, root=None):
    """Reverse the HW arch string to a build target by enumerating boards/.

    cmake/px4_config.cmake uppercases the target and turns '-' into '_' to
    produce the HW arch, so match by applying the same transform. Returns
    (target, None) on a unique match, (None, reason) otherwise.
    """
    if not hw_arch:
        return None, 'board did not report a HW arch'
    matches = [t for t in list_board_targets(root)
               if t.upper().replace('-', '_') == hw_arch.strip().upper()]
    if len(matches) == 1:
        return matches[0], None
    if not matches:
        return None, 'no boards/ entry maps to HW arch {!r}'.format(hw_arch)
    return None, 'HW arch {!r} is ambiguous: {}'.format(hw_arch, ', '.join(matches))


def split_target(target, root=None):
    """Resolve a build target to (base_target, label, board_dir).

    Accepts labeled build targets too (px4_fmu-v6xrt_bench resolves to the
    px4_fmu-v6xrt board dir with label 'bench'): the label selects a
    .px4board config within the same board. Bare targets get label
    'default'. Returns (None, None, None) if no boards/ entry matches.
    """
    targets = list_board_targets(root)
    if target in targets:
        return target, 'default', targets[target]
    base = [t for t in targets if target.startswith(t + '_')]
    if base:
        base_target = max(base, key=len)
        label = target[len(base_target) + 1:]
        return base_target, label, targets[base_target]
    return None, None, None


def board_id_for_target(target, root=None):
    """Numeric board_id from boards/<vendor>/<model>/firmware.prototype.

    Labeled build targets resolve to the same board dir; the board_id is
    unchanged by the label.
    """
    _, _, mdir = split_target(target, root)
    if mdir is None:
        return None
    proto = os.path.join(mdir, 'firmware.prototype')
    try:
        with open(proto) as f:
            return json.load(f).get('board_id')
    except (OSError, ValueError):
        return None


def check_board_id(meta, target, root=None):
    """Early wrong-board check: .px4 board_id vs the connected board's target.

    Returns (ok, detail). ok is None when the check cannot be performed
    (unknown target or no prototype); the uploader still enforces board_id
    against the bootloader at flash time.
    """
    if not target:
        return None, 'board target unknown; deferring board_id check to the uploader'
    proto_id = board_id_for_target(target, root)
    if proto_id is None:
        return None, 'no firmware.prototype board_id for {}; deferring to the uploader'.format(target)
    if meta['board_id'] == proto_id:
        return True, 'board_id {} matches {}'.format(meta['board_id'], target)
    return False, 'firmware board_id {} does not match {} (board_id {})'.format(
        meta['board_id'], target, proto_id)


def _stream_uploader(proc, state, interactive):
    """Echo uploader output; hint the operator if the reboot loop spins.

    A board with a wedged mavlink cannot be soft-rebooted into the
    bootloader (hit live on the bench); the fix is a USB replug so the
    uploader catches the bootloader at power-on.
    """
    for line in proc.stdout:
        line = line.rstrip('\n')
        print('  [uploader] {}'.format(line), flush=True)
        if 'Attempting reboot' in line:
            state['reboot_attempts'] += 1
            if (interactive and not state['hinted']
                    and state['reboot_attempts'] >= REBOOT_HINT_AFTER_ATTEMPTS):
                state['hinted'] = True
                print('\n  OPERATOR: the board is not responding to the soft '
                      'reboot request (wedged mavlink cannot reboot itself).\n'
                      '  UNPLUG and REPLUG the USB cable now; the uploader '
                      'will catch the bootloader at power-on.\n', flush=True)


def flash_firmware(px4_path, connection, baud=DEFAULT_BAUD, interactive=True,
                   timeout=FLASH_TIMEOUT_S, mav=None):
    """Flash a .px4 via Tools/px4_uploader.py and reconnect.

    Closes the given mav connection first (the uploader needs the port),
    streams uploader output with an overall timeout, then waits for USB
    re-enumeration plus a fresh heartbeat. Returns (new_mav, None) or
    (None, error_string).
    """
    if not is_serial_device(connection):
        return None, 'flashing requires a serial device connection, got {!r}'.format(connection)

    if mav is not None:
        try:
            mav.close()
        except Exception:
            pass
        time.sleep(0.5)

    uploader = os.path.join(repo_root(), 'Tools', 'px4_uploader.py')
    if not os.path.exists(uploader):
        return None, 'uploader not found: {}'.format(uploader)

    cmd = [sys.executable, uploader, px4_path,
           '--port', connection, '--baud-flightstack', str(baud)]
    print('[INFO] flashing: {}'.format(' '.join(cmd)), flush=True)

    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
    except OSError as e:
        return None, 'could not launch uploader: {}'.format(e)

    state = {'reboot_attempts': 0, 'hinted': False}
    reader = threading.Thread(target=_stream_uploader,
                              args=(proc, state, interactive), daemon=True)
    reader.start()
    try:
        rc = proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        return None, 'uploader hung (killed after {:.0f}s, {} reboot attempts seen)'.format(
            timeout, state['reboot_attempts'])
    reader.join(timeout=5)

    if rc != 0:
        return None, 'uploader exited with code {}'.format(rc)

    print('[INFO] upload done; waiting for the board to boot', flush=True)
    try:
        newmav, elapsed = wait_reconnect(connection, baud=baud, timeout=90)
    except TimeoutError as e:
        return None, 'board did not come back after flashing: {}'.format(e)
    print('[INFO] board back after {:.1f}s'.format(elapsed), flush=True)
    return newmav, None


def build_firmware(target, timeout=BUILD_TIMEOUT_S, root=None):
    """Run `make <target>` at the repo root; return (artifact_path, None).

    Output is streamed. A cold NuttX build can exceed 10 minutes, hence the
    generous default timeout. Returns (None, error_string) on failure.
    """
    root = root or repo_root()
    print('[INFO] building: make {} (in {})'.format(target, root), flush=True)
    try:
        proc = subprocess.Popen(['make', target], cwd=root)
    except OSError as e:
        return None, 'could not launch make: {}'.format(e)
    try:
        rc = proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        return None, 'build timed out after {:.0f}s (see --build-timeout)'.format(timeout)
    if rc != 0:
        return None, 'make {} failed with exit code {}'.format(target, rc)

    candidates = [os.path.join(root, 'build', target, target + '.px4'),
                  os.path.join(root, 'build', target + '_default',
                               target + '_default.px4')]
    for path in candidates:
        if os.path.exists(path):
            return path, None
    return None, 'build succeeded but no artifact at {}'.format(' or '.join(candidates))


def source_tree_hash(root=None):
    """git rev-parse HEAD of this checkout, or None."""
    try:
        out = subprocess.run(['git', 'rev-parse', 'HEAD'],
                             cwd=root or repo_root(), capture_output=True,
                             text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def download_release(tag, target, dest_dir, timeout=DOWNLOAD_TIMEOUT_S):
    """Download <target>_default.px4 from a GitHub release via gh.

    tag is a release tag like v1.17.0, or 'latest'. Returns
    (px4_path, None) or (None, error_string).
    """
    if shutil.which('gh') is None:
        return None, "the 'gh' CLI is required for --release (brew install gh)"
    asset = '{}_default.px4'.format(target)
    cmd = ['gh', 'release', 'download', '--repo', GITHUB_REPO,
           '--pattern', asset, '--dir', dest_dir, '--clobber']
    if tag and tag != 'latest':
        cmd.insert(3, tag)
    print('[INFO] downloading: {}'.format(' '.join(cmd)), flush=True)
    try:
        proc = subprocess.run(cmd, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None, 'gh release download timed out after {:.0f}s'.format(timeout)
    except OSError as e:
        return None, 'could not launch gh: {}'.format(e)
    if proc.returncode != 0:
        return None, 'gh release download failed (tag {!r}, asset {!r})'.format(tag, asset)
    path = os.path.join(dest_dir, asset)
    if not os.path.exists(path):
        return None, 'gh reported success but {} is missing'.format(path)
    return path, None


# Bench-relevant board config options and the suite tests they enable.
# All four are 'default n' in their Kconfig, so a '=y' line in the merged
# board config (default.px4board plus the label overlay, see
# cmake/kconfig.cmake:47-54) decides whether the image contains them.
BENCH_CAPABILITIES = (
    ('CONFIG_MODULES_SIMULATION_SIMULATOR_SIH', 'simulator_sih', 'sih/flight_mission'),
    ('CONFIG_SYSTEMCMDS_SD_BENCH', 'sd_bench', 'bench/storage_stress (bench phase)'),
    ('CONFIG_SYSTEMCMDS_SD_STRESS', 'sd_stress', 'bench/storage_stress (stress phase)'),
    ('CONFIG_SYSTEMCMDS_SERIAL_TEST', 'serial_test', 'bench/serial_loopback'),
)


def board_config_files(target, root=None):
    """Config files the build of `target` merges, in merge order.

    Non-default labels merge default.px4board plus <label>.px4board
    (cmake/kconfig.cmake:47-54). Returns [] for unknown targets.
    """
    base, label, mdir = split_target(target, root)
    if mdir is None:
        return []
    files = [os.path.join(mdir, 'default.px4board')]
    if label != 'default':
        files.append(os.path.join(mdir, label + '.px4board'))
    return [f for f in files if os.path.exists(f)]


def _options_in_file(path):
    """CONFIG_* names set to y in one .px4board file."""
    options = set()
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                m = re.match(r'^(CONFIG_[A-Za-z0-9_]+)=y$', line)
                if m:
                    options.add(m.group(1))
    except OSError:
        pass
    return options


def board_config_options(target, root=None):
    """All CONFIG_* options set to y in the merged config for `target`."""
    options = set()
    for path in board_config_files(target, root):
        options |= _options_in_file(path)
    return options


def capability_report(target, root=None):
    """Which bench-relevant options the build of `target` will contain.

    Returns a list of (option, command, test, enabled) tuples following
    BENCH_CAPABILITIES.
    """
    enabled = board_config_options(target, root)
    return [(opt, cmd, test, opt in enabled)
            for opt, cmd, test in BENCH_CAPABILITIES]


def variants_with_option(target, option, root=None):
    """Sibling variants of the same board whose merged config enables option.

    Scans the board dir's *.px4board files; a labeled variant inherits
    default.px4board, so it has the option if either file sets it. Returns
    full build targets (e.g. px4_fmu-v6xrt_bench), excluding `target` itself.
    """
    base, _, mdir = split_target(target, root)
    if mdir is None:
        return []
    default_has = option in _options_in_file(os.path.join(mdir, 'default.px4board'))
    variants = []
    for fname in sorted(os.listdir(mdir)):
        if not fname.endswith('.px4board'):
            continue
        label = fname[:-len('.px4board')]
        if default_has or option in _options_in_file(os.path.join(mdir, fname)):
            full = base if label == 'default' else '{}_{}'.format(base, label)
            if full != target:
                variants.append(full)
    return variants
