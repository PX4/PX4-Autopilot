#!/usr/bin/env python3
"""
Reboot torture test for a PX4 board on real NuttX hardware.

The v1.18 concurrency and locking rework changed startup ordering and shared
state teardown, so the risk is a board that boots fine once but occasionally
hangs on reboot, or comes back with a shell that never responds. This script
reboots the board in a loop and, after each reboot, proves the system is alive
two ways: a fresh MAVLink heartbeat and an nsh command (`free`) that actually
completes over SERIAL_CONTROL.

Every reconnect has a hard timeout. If a cycle does not come back, we do not
hang: we report the cycle as FAIL naming what stalled, abort the remaining
cycles (the board is gone, nothing to gain by waiting again), print the
summary, and exit non-zero.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import px4bench


def main():
    parser = argparse.ArgumentParser(
        description='PX4 reboot torture test: reboot N times, prove liveness each cycle.')
    px4bench.add_connection_args(parser)
    parser.add_argument('--iterations', type=int, default=10,
                        help='number of reboot cycles (default: %(default)s)')
    parser.add_argument('--cycle-timeout', type=float, default=60,
                        help='seconds to allow per reboot+reconnect cycle (default: %(default)s)')
    args = parser.parse_args()

    report = px4bench.Reporter('reboot_loop')

    try:
        mav = px4bench.connect(args.connection, baud=args.baudrate,
                               timeout=args.connect_timeout)
    except (TimeoutError, OSError) as e:
        report.fail('initial connect', str(e))
        sys.exit(report.finish())
    report.ok('initial connect', 'heartbeat from {}'.format(args.connection))

    conn_str = args.connection
    for i in range(1, args.iterations + 1):
        try:
            newmav, elapsed = px4bench.reboot_and_reconnect(
                mav, conn_str, baud=args.baudrate, timeout=args.cycle_timeout)
        except TimeoutError as e:
            report.fail('cycle {}'.format(i), str(e))
            report.info('board did not return; aborting remaining {} cycle(s)'.format(
                args.iterations - i))
            sys.exit(report.finish())

        mav = newmav

        # confirm a live heartbeat after reconnect
        hb = px4bench.wait_heartbeat(mav, timeout=5)
        if hb is None:
            report.fail('cycle {} heartbeat'.format(i),
                        'reconnected but no heartbeat within 5s')
            report.info('aborting remaining {} cycle(s)'.format(args.iterations - i))
            sys.exit(report.finish())

        # prove the shell (and thus the running system) is responsive
        shell = px4bench.MavlinkShell(mav)
        if not shell.open(timeout=5):
            report.fail('cycle {} shell'.format(i),
                        'no nsh output on SERIAL_CONTROL within 5s')
            shell.close()
            report.info('aborting remaining {} cycle(s)'.format(args.iterations - i))
            sys.exit(report.finish())

        try:
            _, timed_out = shell.run('free', timeout=10)
        finally:
            shell.close()
        if timed_out:
            report.fail('cycle {}'.format(i),
                        'shell command free stalled (no completion within 10s)')
            report.info('aborting remaining {} cycle(s)'.format(args.iterations - i))
            sys.exit(report.finish())

        report.ok('cycle {}'.format(i),
                  'reconnected in {:.1f}s, shell responsive'.format(elapsed))

    mav.close()
    sys.exit(report.finish())


if __name__ == '__main__':
    main()
