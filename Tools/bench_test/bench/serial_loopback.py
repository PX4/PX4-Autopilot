#!/usr/bin/env python3
"""
UART loopback test using the firmware's serial_test command.

Operator/fixture test: a loopback jumper (TX wired to RX) is installed on
one of the board's UARTs, then serial_test (src/systemcmds/serial_test)
transmits a known pattern and verifies its own received bytes through the
jumper. This exercises the real UART driver, DMA, and interrupt path,
which the USB-only bench tests never touch. Intended for manufacturing
end-of-line checks and for qualifying serial driver changes.

NOT part of the default suite: it needs a physical fixture. Like
usb_replug, run it manually.

The chosen UART must not be claimed by a running service (mavlink
instance, GPS driver, RC input). Pick a port whose function is disabled in
the parameters, or a dedicated test pad on the fixture.

Verified against src/systemcmds/serial_test/serial_test.c: options
-p PORT -b BAUD -o TX_SECONDS -i RX_SECONDS; the session summary line is
'<port>: count for this session: rx=N, tx=N, rx err=N'; each received
byte that does not match the expected pattern prints an
'Error, count: ...' line.
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import px4bench

SUMMARY_RE = re.compile(
    r'count for this session:\s*rx=(\d+),\s*tx=(\d+),\s*rx err=(\d+)')


def main():
    parser = argparse.ArgumentParser(
        description='UART loopback test via serial_test over the MAVLink shell '
                    '(requires a TX-RX jumper on the tested port).')
    px4bench.add_connection_args(parser)
    parser.add_argument('--device', required=True,
                        help="UART device ON the board to test, e.g. /dev/ttyS2 "
                             "(must have the loopback jumper and no service "
                             "using it)")
    parser.add_argument('--test-baud', type=int, default=115200,
                        help='baud rate for the loopback run (default: %(default)s)')
    parser.add_argument('--seconds', type=int, default=5,
                        help='transmit/receive duration (default: %(default)s)')
    parser.add_argument('--no-prompt', action='store_true', default=False,
                        help='skip the jumper confirmation pause (fixture automation)')
    parser.add_argument('--report-dir', default='bench_reports',
                        help='base directory for report output (default: %(default)s)')
    args = parser.parse_args()

    report = px4bench.Reporter('serial_loopback')
    report_dir = px4bench.make_report_dir(args.report_dir, 'serial_loopback')
    report.info('report dir: {}'.format(report_dir))

    try:
        mav = px4bench.connect(args.connection, baud=args.baudrate,
                               timeout=args.connect_timeout)
    except (TimeoutError, OSError) as e:
        report.fail('connect', str(e))
        sys.exit(report.finish())
    report.ok('connect', 'heartbeat from {}'.format(args.connection))

    shell = px4bench.MavlinkShell(mav)
    if not shell.open(timeout=5):
        report.fail('shell open', 'no nsh output on SERIAL_CONTROL within 5s')
        mav.close()
        sys.exit(report.finish())

    present, _ = px4bench.shell_command_exists(shell, 'serial_test -h')
    if present is None:
        report.fail('probe', 'serial_test probe stalled over the shell')
        shell.close()
        mav.close()
        sys.exit(report.finish())
    if present is False:
        report.info('WARNING: serial_test not in this firmware (command not '
                    'found); skipping. Enable CONFIG_SYSTEMCMDS_SERIAL_TEST=y')
        shell.close()
        mav.close()
        sys.exit(px4bench.EXIT_SKIP)

    if not args.no_prompt:
        print('-' * 60)
        print('Install the loopback jumper now: wire TX to RX on {}.'.format(
            args.device))
        print('Make sure no service (mavlink, GPS, RC) is configured on that '
              'port.')
        print('-' * 60, flush=True)
        input('Press Enter when the jumper is installed... ')

    cmd = 'serial_test -p {} -b {} -o {} -i {}'.format(
        args.device, args.test_baud, args.seconds, args.seconds)
    timeout = args.seconds + 20
    report.info('running {} (timeout {:.0f}s)'.format(cmd, timeout))
    out, timed_out = shell.run(cmd, timeout=timeout)
    try:
        with open(os.path.join(report_dir, 'serial_test.txt'), 'w') as f:
            f.write(out)
    except OSError:
        pass
    shell.close()
    mav.close()

    if timed_out:
        report.fail('serial_test', 'serial_test stalled (no completion within '
                                   '{:.0f}s), output saved'.format(timeout))
        sys.exit(report.finish())

    if 'ERROR: Port argument required' in out or 'Cannot open' in out.replace("Can't", 'Cannot'):
        report.fail('serial_test', 'could not open {} (in use or missing?): '
                                   'see saved output'.format(args.device))
        sys.exit(report.finish())

    m = SUMMARY_RE.search(out)
    if not m:
        report.fail('serial_test summary',
                    'no session summary in output (port in use or wrong '
                    'device?); output saved')
        sys.exit(report.finish())
    rx, tx, err = int(m.group(1)), int(m.group(2)), int(m.group(3))
    mismatches = len(re.findall(r'Error, count:', out))

    report.check('tx', tx > 0, '{} bytes transmitted'.format(tx))
    report.check('rx', rx > 0,
                 '{} bytes received through the jumper'.format(rx)
                 if rx > 0 else 'nothing received: jumper missing or wrong port?')
    report.check('pattern errors', err == 0 and mismatches == 0,
                 'rx err={}, {} mismatch line(s)'.format(err, mismatches))
    delta = abs(tx - rx)
    tolerance = max(1024, tx // 50)
    report.check('rx/tx balance', delta <= tolerance,
                 'tx={} rx={} delta={} (tolerance {})'.format(tx, rx, delta, tolerance))

    sys.exit(report.finish())


if __name__ == '__main__':
    main()
