#!/usr/bin/env python3
"""
Orchestrator for the non-interactive PX4 v1.18 bench-test suite.

Runs the individual bench tests in sequence against a board on the bench,
each as its own subprocess so a hang in one test cannot wedge the whole
run. The suite exists to qualify v1.18 on real NuttX hardware after the
mavlink locking rework; the individual tests exercise boot, parameter and
mission traffic, the logger + MAVFTP path, reboot recovery, and (when a
second link is given) the dual-link nested-send lock stress.

Each test enforces its own timeouts, and this orchestrator wraps each in a
--per-test-timeout so even a fully wedged child is killed and recorded as a
FAIL naming what hung. usb_replug is interactive and is never run here.

Before any test starts, the firmware gate establishes exactly which build
is on the board (and can flash one): qualification means knowing what you
tested. The identity is printed, written to firmware.json in the suite
report dir, and stamped into every test's report dir.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from typing import NoReturn

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import px4bench
from px4bench import firmware


# Test order for the suite. link_forwarding is inserted after
# mission_stress only when a second connection is provided.
BASE_SEQUENCE = [
    'boot_health',
    'param_stress',
    'mission_stress',
    'storage_stress',
    'log_transfer',
    'reboot_loop',
    'flight_mission',
]
DUAL_LINK_TEST = 'link_forwarding'
DUAL_LINK_AFTER = 'mission_stress'

# every test lives in bench/ except the simulated flight
SCRIPT_SUBDIR = {'flight_mission': 'sih'}


def script_path(name):
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, SCRIPT_SUBDIR.get(name, 'bench'), name + '.py')


def build_argv(name, args):
    """Build the argv for one test's subprocess, per each test's own CLI."""
    argv = [sys.executable, script_path(name), args.connection]

    # tests that take a second positional connection
    if name in ('mission_stress', DUAL_LINK_TEST):
        if args.connection2:
            argv.append(args.connection2)

    # baud rate is accepted by every test
    argv += ['-b', str(args.baudrate)]

    # per-test extra flags
    if name in ('boot_health', 'log_transfer', 'storage_stress', 'flight_mission'):
        argv += ['--report-dir', args.report_dir]
    if name == 'flight_mission' and args.allow_arming:
        argv += ['--allow-arming']

    return argv


def run_one(name, args):
    """Run one test as a subprocess with inherited stdout/stderr so its output
    streams live. Enforces --per-test-timeout; on expiry kills the whole
    process group. Returns (status, duration_seconds, detail).
    """
    argv = build_argv(name, args)
    print('=' * 70, flush=True)
    print('RUN {}: {}'.format(name, ' '.join(argv)), flush=True)
    print('=' * 70, flush=True)

    start = time.monotonic()
    # start_new_session so the child gets its own process group; if it hangs we
    # can kill the whole group (a wedged test may have spawned threads/children).
    try:
        proc = subprocess.Popen(argv, start_new_session=True)
    except OSError as e:
        return 'FAIL', 0.0, 'could not launch: {}'.format(e)

    try:
        rc = proc.wait(timeout=args.per_test_timeout)
    except subprocess.TimeoutExpired:
        # kill the whole process group, escalate to SIGKILL if needed
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except OSError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except OSError:
                pass
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
        dur = time.monotonic() - start
        return 'FAIL', dur, '{} hung (killed after {:.0f}s)'.format(name, args.per_test_timeout)

    dur = time.monotonic() - start
    if rc == 0:
        return 'PASS', dur, ''
    if rc == px4bench.EXIT_SKIP:
        return 'SKIP', dur, 'skipped by the test (see its output above)'
    return 'FAIL', dur, 'exit code {}'.format(rc)


def gate_abort(report, code=1) -> NoReturn:
    """Print the preflight summary and abort the suite."""
    report.finish()
    print('firmware gate failed; no test was run.', flush=True)
    sys.exit(code)


def flash_and_verify(report, args, mav, ident, px4_path, source):
    """Common tail for every firmware source: check board_id, flash, verify.

    Returns (new_mav, new_identity); aborts the suite on any failure.
    """
    try:
        meta = firmware.parse_px4_metadata(px4_path)
    except ValueError as e:
        report.fail('firmware_file', str(e))
        gate_abort(report, 2)
    report.ok('firmware_file', '{} ({}, board_id {})'.format(
        px4_path, meta.get('git_identity') or 'no git identity', meta['board_id']))

    # Early wrong-board check against boards/<vendor>/<model>/firmware.prototype;
    # the uploader still enforces board_id against the bootloader at flash time.
    target = args.target
    if not target:
        target, terr = firmware.infer_build_target(ident.get('hw_arch'))
        if target is None:
            report.info('board_id preflight check skipped: {}'.format(terr))
    id_ok, id_detail = firmware.check_board_id(meta, target)
    if id_ok is False:
        report.fail('firmware_board_id', id_detail)
        gate_abort(report)
    elif id_ok is None:
        report.info(id_detail)
    else:
        report.ok('firmware_board_id', id_detail)

    newmav, err = firmware.flash_firmware(
        px4_path, args.connection, baud=args.baudrate,
        interactive=sys.stdin.isatty(), timeout=args.flash_timeout, mav=mav)
    if newmav is None:
        report.fail('firmware_flash', err)
        gate_abort(report)
    report.ok('firmware_flash', source)

    expected = firmware.expected_hash_from_metadata(meta)
    ok, new_ident = firmware.verify_identity(report, newmav, expected)
    if not ok or new_ident is None:
        gate_abort(report)
    return newmav, new_ident


def report_build_capabilities(report, target, interactive):
    """Bench capability report for a --build target: which bench-relevant
    modules the image will contain and which suite tests will run or skip.

    For anything missing: name a sibling variant that has it, show the
    exact config line, and (interactive only) offer to append the line to
    the local board config. Any edit is local and uncommitted.
    """
    rows = firmware.capability_report(target)
    print('-' * 70)
    print('BENCH CAPABILITIES OF BUILD {}'.format(target))
    for opt, cmd, test, enabled in rows:
        print('  {:<44} {:<9} {}'.format(
            opt, 'yes' if enabled else 'MISSING',
            '{} will run'.format(test) if enabled
            else '{} will be skipped'.format(test)))
    print('-' * 70, flush=True)

    missing = [(opt, cmd, test) for opt, cmd, test, enabled in rows if not enabled]
    if not missing:
        report.ok('build_capabilities',
                  '{}: all bench-relevant modules included'.format(target))
        return

    report.info('this build will not include: {}; the dependent tests will '
                'be skipped'.format(', '.join(cmd for _, cmd, _ in missing)))
    config_files = firmware.board_config_files(target)
    edit_file = config_files[-1] if config_files else None
    for opt, cmd, test in missing:
        variants = firmware.variants_with_option(target, opt)
        if variants:
            print('  {}: use a variant that includes it, e.g. --target {}'.format(
                cmd, variants[0]), flush=True)
        if edit_file:
            print('  {}: or add this line to {}:'.format(cmd, edit_file))
            print('      {}=y'.format(opt), flush=True)

    if interactive and edit_file:
        answer = input('Append the missing line(s) to {} now? [y/N] '.format(
            edit_file)).strip().lower()
        if answer == 'y':
            try:
                with open(edit_file, 'a') as f:
                    for opt, _, _ in missing:
                        f.write('{}=y\n'.format(opt))
                print('appended {} line(s) to {}.'.format(len(missing), edit_file))
                print('NOTE: this change is LOCAL and UNCOMMITTED; commit it '
                      'yourself for it to stick.', flush=True)
            except OSError as e:
                print('could not edit {}: {}'.format(edit_file, e), flush=True)


def resolve_target(report, args, ident):
    """--target, or inferred from the board's HW arch; aborts if ambiguous."""
    if args.target:
        return args.target
    target, terr = firmware.infer_build_target(ident.get('hw_arch'))
    if target is None:
        report.fail('firmware_target',
                    '{}; pass --target (e.g. --target px4_fmu-v6xrt)'.format(terr))
        gate_abort(report, 2)
    report.info('board target inferred from HW arch: {}'.format(target))
    return target


def preflight(args, suite_dir):
    """Firmware gate: establish (and optionally control) the build under test.

    Returns the final identity dict. Exits on any gate failure; no test
    runs against an unknown or unintended build.
    """
    report = px4bench.Reporter('preflight')

    gate_flag_given = any((args.any_firmware, args.expect_hash, args.firmware,
                           args.build, args.release))
    if not gate_flag_given and not sys.stdin.isatty():
        report.fail('firmware_gate',
                    'no firmware expectation given and stdin is not a TTY. '
                    'Automation must state what it is testing: pass one of '
                    '--firmware FILE, --build, --release TAG, --expect-hash PREFIX, '
                    'or --any-firmware.')
        gate_abort(report, 2)

    try:
        mav = px4bench.connect(args.connection, baud=args.baudrate)
    except (TimeoutError, OSError) as e:
        report.fail('connect', str(e))
        gate_abort(report, 2)

    ident, err = firmware.board_identity(mav)
    if ident is None:
        report.fail('board_identity', err)
        gate_abort(report, 2)

    print('=' * 70)
    print('FIRMWARE ON BOARD ({})'.format(args.connection))
    print(firmware.format_identity(ident))
    print('=' * 70, flush=True)

    source = 'kept firmware already on the board'

    if args.any_firmware:
        report.ok('firmware_identity',
                  'any-firmware: testing {} as-is'.format(ident['git_hash']))

    elif args.expect_hash:
        ok = firmware.hashes_match(ident['git_hash'], args.expect_hash)
        report.check('firmware_identity', ok,
                     'board reports {} , expected {}'.format(
                         ident['git_hash'], args.expect_hash))
        if not ok:
            mav.close()
            gate_abort(report)

    elif args.firmware:
        mav, ident = flash_and_verify(report, args, mav, ident, args.firmware,
                                      'flashed local file {}'.format(args.firmware))
        source = 'flashed local file {}'.format(args.firmware)

    elif args.build:
        target = resolve_target(report, args, ident)
        report_build_capabilities(report, target, sys.stdin.isatty())
        px4_path, err = firmware.build_firmware(target, timeout=args.build_timeout)
        if px4_path is None:
            report.fail('firmware_build', err)
            gate_abort(report)
        report.ok('firmware_build', px4_path)
        head = firmware.source_tree_hash()
        meta = firmware.parse_px4_metadata(px4_path)
        if head and meta.get('git_hash') and not firmware.hashes_match(meta['git_hash'], head):
            report.info('WARNING: artifact hash {} differs from source tree HEAD {}; '
                        'stale build?'.format(meta['git_hash'], head))
        mav, ident = flash_and_verify(report, args, mav, ident, px4_path,
                                      'built and flashed {} from this tree'.format(target))
        source = 'built {} from source tree (HEAD {})'.format(target, head or 'unknown')

    elif args.release:
        target = resolve_target(report, args, ident)
        px4_path, err = firmware.download_release(args.release, target, suite_dir)
        if px4_path is None:
            report.fail('firmware_download', err)
            gate_abort(report)
        report.ok('firmware_download', px4_path)
        mav, ident = flash_and_verify(report, args, mav, ident, px4_path,
                                      'flashed release {} ({})'.format(args.release, target))
        source = 'GitHub release {} ({})'.format(args.release, target)

    else:
        mav, ident, source = interactive_gate(report, args, mav, ident, suite_dir)

    fw_info = {k: v for k, v in ident.items() if k != 'raw'}
    fw_info['source'] = source
    firmware.stamp_identity(suite_dir, ident, extra={'source': source})
    os.environ[px4bench.FIRMWARE_INFO_ENV] = json.dumps(fw_info, sort_keys=True)

    try:
        mav.close()
    except Exception:
        pass

    if report.finish() != 0:
        print('firmware gate failed; no test was run.', flush=True)
        sys.exit(1)
    print()
    return ident


def interactive_gate(report, args, mav, ident, suite_dir):
    """Operator menu when no firmware flag was given on a TTY."""
    inferred, _ = firmware.infer_build_target(ident.get('hw_arch'))
    build_label = inferred if inferred else '<target unknown, will prompt>'
    while True:
        print('\nNo firmware expectation given. Choose:')
        print('  [1] continue on the current firmware ({})'.format(ident['git_hash'][:12]))
        print('  [2] flash a local .px4 file')
        print('  [3] build {} from this source tree and flash'.format(build_label))
        print('  [4] download a GitHub release and flash')
        print('  [q] abort')
        choice = input('> ').strip().lower()

        if choice == '1':
            report.ok('firmware_identity',
                      'operator kept current firmware {}'.format(ident['git_hash']))
            return mav, ident, 'operator kept firmware already on the board'

        if choice == '2':
            path = input('.px4 path: ').strip()
            if not path or not os.path.exists(path):
                print('no such file: {!r}'.format(path))
                continue
            mav, ident = flash_and_verify(report, args, mav, ident, path,
                                          'flashed local file {}'.format(path))
            return mav, ident, 'flashed local file {}'.format(path)

        if choice == '3':
            target = args.target or inferred
            if not target:
                target = input('build target (e.g. px4_fmu-v6xrt): ').strip()
                if not target:
                    continue
            report_build_capabilities(report, target, True)
            px4_path, err = firmware.build_firmware(target, timeout=args.build_timeout)
            if px4_path is None:
                report.fail('firmware_build', err)
                gate_abort(report)
            report.ok('firmware_build', px4_path)
            mav, ident = flash_and_verify(report, args, mav, ident, px4_path,
                                          'built and flashed {}'.format(target))
            return mav, ident, 'built {} from source tree'.format(target)

        if choice == '4':
            target = args.target or inferred
            if not target:
                target = input('board target (e.g. px4_fmu-v6xrt): ').strip()
                if not target:
                    continue
            tag = input('release tag [latest]: ').strip() or 'latest'
            px4_path, err = firmware.download_release(tag, target, suite_dir)
            if px4_path is None:
                report.fail('firmware_download', err)
                gate_abort(report)
            report.ok('firmware_download', px4_path)
            mav, ident = flash_and_verify(report, args, mav, ident, px4_path,
                                          'flashed release {} ({})'.format(tag, target))
            return mav, ident, 'GitHub release {} ({})'.format(tag, target)

        if choice == 'q':
            report.fail('firmware_gate', 'aborted by operator')
            gate_abort(report, 2)

        print('unrecognized choice: {!r}'.format(choice))


def main():
    parser = argparse.ArgumentParser(
        description='Run the non-interactive PX4 v1.18 bench-test suite.')
    parser.add_argument('connection',
                        help='MAVLink connection: serial device, udp:IP:PORT, or tcp:IP:PORT')
    parser.add_argument('connection2', nargs='?', default=None,
                        help='optional second connection; enables link_forwarding '
                             'and is passed to mission_stress')
    parser.add_argument('--baudrate', '-b', type=int, default=57600,
                        help='serial baud rate (default: %(default)s)')
    parser.add_argument('--skip', default='',
                        help='comma-separated list of test names to skip')
    parser.add_argument('--report-dir', default='bench_reports',
                        help='base directory for reports (default: %(default)s)')
    parser.add_argument('--per-test-timeout', type=float, default=900,
                        help='seconds before a test is killed as hung (default: %(default)s)')
    parser.add_argument('--stop-on-fail', action='store_true', default=False,
                        help='stop the suite at the first failing test')
    parser.add_argument('--allow-arming', action='store_true', default=False,
                        help='skip the arming confirmation before the simulated '
                             'flight test (automation/manufacturing; board must '
                             'be bare, nothing on the output rails)')

    gate = parser.add_mutually_exclusive_group()
    gate.add_argument('--firmware', metavar='FILE',
                      help='flash this .px4 before testing, then verify identity')
    gate.add_argument('--build', action='store_true',
                      help='build the board target from this source tree, flash, verify')
    gate.add_argument('--release', metavar='TAG',
                      help="flash a GitHub release artifact (a tag like v1.17.0, or 'latest')")
    gate.add_argument('--expect-hash', metavar='PREFIX',
                      help='verify the board already runs this git hash (prefix match), no flash')
    gate.add_argument('--any-firmware', action='store_true',
                      help='explicitly proceed with whatever firmware is on the board')
    parser.add_argument('--target', default=None,
                        help='board target for --build/--release (e.g. px4_fmu-v6xrt); '
                             'inferred from the connected board when possible')
    parser.add_argument('--build-timeout', type=float, default=1800,
                        help='seconds before --build is killed (default: %(default)s)')
    parser.add_argument('--flash-timeout', type=float, default=600,
                        help='seconds before the uploader is killed (default: %(default)s)')
    args = parser.parse_args()

    skip = {s.strip() for s in args.skip.split(',') if s.strip()}

    suite_dir = px4bench.make_report_dir(args.report_dir, 'suite')
    print('suite report dir: {}'.format(suite_dir))
    preflight(args, suite_dir)

    # build the run sequence
    sequence = []
    for name in BASE_SEQUENCE:
        sequence.append(name)
        if name == DUAL_LINK_AFTER and args.connection2:
            sequence.append(DUAL_LINK_TEST)

    print('PX4 bench-test suite')
    print('  connection : {}'.format(args.connection))
    print('  connection2: {}'.format(args.connection2 if args.connection2 else '(none)'))
    print('  sequence   : {}'.format(', '.join(sequence)))
    if skip:
        print('  skipping   : {}'.format(', '.join(sorted(skip))))
    print('  NOTE: usb_replug (USB replug) and serial_loopback (loopback '
          'jumper) need an operator or fixture; run them manually.')
    if 'flight_mission' not in skip and not args.allow_arming:
        print('  NOTE: flight_mission arms the board (simulated flight); you '
              'will be asked to confirm, or pass --allow-arming.')
    print()

    results = []  # (name, status, duration, detail)
    for name in sequence:
        if name in skip:
            results.append((name, 'SKIP', 0.0, 'skipped via --skip'))
            print('SKIP {}'.format(name), flush=True)
            continue
        status, dur, detail = run_one(name, args)
        results.append((name, status, dur, detail))
        print('--> {} {} ({:.1f}s){}'.format(
            name, status, dur, ' - ' + detail if detail else ''), flush=True)
        if status == 'FAIL' and args.stop_on_fail:
            print('stop-on-fail set; halting suite after {}'.format(name), flush=True)
            break

    # summary table
    print()
    print('=' * 70)
    print('BENCH SUITE SUMMARY')
    print('=' * 70)
    name_w = max([len('TEST')] + [len(r[0]) for r in results])
    print('{:<{w}}  {:<6}  {:>8}  {}'.format('TEST', 'STATUS', 'TIME(s)', 'DETAIL', w=name_w))
    print('-' * 70)
    n_pass = n_fail = n_skip = 0
    for name, status, dur, detail in results:
        if status == 'PASS':
            n_pass += 1
        elif status == 'FAIL':
            n_fail += 1
        else:
            n_skip += 1
        print('{:<{w}}  {:<6}  {:>8.1f}  {}'.format(name, status, dur, detail, w=name_w))
    print('-' * 70)
    print('{} passed, {} failed, {} skipped'.format(n_pass, n_fail, n_skip))

    sys.exit(1 if n_fail > 0 else 0)


if __name__ == '__main__':
    main()
