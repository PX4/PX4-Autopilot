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
"""

import argparse
import os
import signal
import subprocess
import sys
import time


# Test order for the suite. dual_link_forwarding is inserted after
# mission_torture only when a second connection is provided.
BASE_SEQUENCE = [
    'boot_health',
    'param_torture',
    'mission_torture',
    'mavftp_log',
    'reboot_loop',
]
DUAL_LINK_TEST = 'dual_link_forwarding'
DUAL_LINK_AFTER = 'mission_torture'


def script_path(name):
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, name + '.py')


def build_argv(name, args):
    """Build the argv for one test's subprocess, per each test's own CLI."""
    argv = [sys.executable, script_path(name), args.connection]

    # tests that take a second positional connection
    if name in ('mission_torture', DUAL_LINK_TEST):
        if args.connection2:
            argv.append(args.connection2)

    # baud rate is accepted by every test
    argv += ['-b', str(args.baudrate)]

    # per-test extra flags
    if name == 'boot_health':
        argv += ['--report-dir', args.report_dir]
    elif name == 'mavftp_log':
        argv += ['--outdir', args.report_dir]

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
    return 'FAIL', dur, 'exit code {}'.format(rc)


def main():
    parser = argparse.ArgumentParser(
        description='Run the non-interactive PX4 v1.18 bench-test suite.')
    parser.add_argument('connection',
                        help='MAVLink connection: serial device, udp:IP:PORT, or tcp:IP:PORT')
    parser.add_argument('connection2', nargs='?', default=None,
                        help='optional second connection; enables dual_link_forwarding '
                             'and is passed to mission_torture')
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
    args = parser.parse_args()

    skip = {s.strip() for s in args.skip.split(',') if s.strip()}

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
    print('  NOTE: usb_replug is interactive and is not run by this suite; '
          'run it manually.')
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
