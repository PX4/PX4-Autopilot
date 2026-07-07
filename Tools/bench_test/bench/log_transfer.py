#!/usr/bin/env python3
"""
Logger + MAVFTP download path test for PX4 v1.18 bench qualification.

The mavlink locking rework touches every message handler that sends a
reply, which includes the FILE_TRANSFER_PROTOCOL handler that backs
MAVFTP. A burst read download is a long sequence of nested send replies,
so a deadlock in the send lock shows up here as a transfer that stops
mid-file. This test starts logging, records for a few seconds, stops and
flushes, then finds and downloads the freshest ULog over MAVFTP and
verifies it.

No arming: `logger on` / `logger off` start and stop logging directly,
overriding arming state, as long as the logger module is running.

Every step is bounded. A stalled transfer is a FAIL that names how many
bytes made it before the stall, not a hung script.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from px4bench import (Reporter, MavlinkShell, add_connection_args, connect,
                      make_report_dir, send_heartbeat)
from px4bench.ftp import LOG_ROOT, ULOG_MAGIC7, ftp_list, ftp_download

from pymavlink import mavftp

SHELL_TIMEOUT = 10.0
MIN_LOG_SIZE = 1024            # a real ULog is more than 1 KB


def main():
    parser = argparse.ArgumentParser(
        description='Logger + MAVFTP download path test (v1.18 bench).')
    add_connection_args(parser)
    parser.add_argument('--log-duration', type=float, default=10,
                        help='seconds to log before stopping (default: %(default)s)')
    parser.add_argument('--report-dir', default='bench_reports',
                        help='base directory for the downloaded log (default: %(default)s)')
    args = parser.parse_args()

    report = Reporter('log_transfer')

    mav = None
    shell = None
    try:
        report.info('Connecting: {} @ {}'.format(args.connection, args.baudrate))
        try:
            mav = connect(args.connection, baud=args.baudrate, timeout=args.connect_timeout)
        except (TimeoutError, OSError) as e:
            report.fail('connect', str(e))
            sys.exit(report.finish())

        # One shell session for the logger commands, torn down in finally so
        # no early exit leaks the firmware nsh task. The FTP phase below uses
        # no shell, so the shell is closed as soon as the logging is flushed.
        shell = MavlinkShell(mav)
        if not shell.open(timeout=5):
            report.fail('shell_open', 'nsh shell did not respond within 5s')
            sys.exit(report.finish())

        # logger status: confirm the module is running before we tell it to log
        out, timed_out = shell.run('logger status', timeout=SHELL_TIMEOUT)
        if timed_out:
            report.fail('logger_status', '`logger status` stalled (no output in {:.0f}s)'.format(
                SHELL_TIMEOUT))
            sys.exit(report.finish())
        low = out.lower()
        if ('not running' in low or 'never' in low or
                (low.strip() == '') or ('running' not in low and 'log' not in low)):
            report.fail('logger_status',
                        'logger module not running (check SDLOG_MODE): {}'.format(out.strip()[:200]))
            sys.exit(report.finish())
        report.ok('logger_status', 'logger module running')

        out, timed_out = shell.run('logger on', timeout=SHELL_TIMEOUT)
        if timed_out:
            report.fail('logger_on', '`logger on` stalled')
            sys.exit(report.finish())
        report.ok('logger_on', 'logging started')

        report.info('Logging for {}s'.format(args.log_duration))
        log_deadline = time.monotonic() + args.log_duration
        while time.monotonic() < log_deadline:
            # keep the link alive and the autopilot streaming during the wait
            send_heartbeat(mav)
            mav.recv_match(blocking=True, timeout=0.5)

        out, timed_out = shell.run('logger off', timeout=SHELL_TIMEOUT)
        if timed_out:
            report.fail('logger_off', '`logger off` stalled (flush not confirmed)')
            sys.exit(report.finish())
        report.ok('logger_off', 'logging stopped and flushed')
        shell.close()
        shell = None
        # give the filesystem a moment to settle after the flush
        time.sleep(1.0)

        # MAVFTP: find the newest log directory, then the newest .ulg in it
        ftp = mavftp.MAVFTP(mav, target_system=mav.target_system,
                            target_component=mav.target_component)

        entries = ftp_list(ftp, LOG_ROOT)
        dirs = sorted([e.name for e in entries if e.is_dir])
        if not dirs:
            report.fail('ftp_list_root',
                        'no log directories under {} (found: {})'.format(
                            LOG_ROOT, [e.name for e in entries]))
            sys.exit(report.finish())
        newest_dir = dirs[-1]
        report.ok('ftp_list_root', 'newest log dir: {}'.format(newest_dir))

        log_dir = '{}/{}'.format(LOG_ROOT, newest_dir)
        entries = ftp_list(ftp, log_dir)
        ulgs = sorted([e for e in entries if not e.is_dir and e.name.endswith('.ulg')],
                      key=lambda e: e.name)
        if not ulgs:
            report.fail('ftp_list_dir',
                        'no .ulg in {} (found: {})'.format(
                            log_dir, [e.name for e in entries]))
            sys.exit(report.finish())
        newest = ulgs[-1]
        report.ok('ftp_list_dir', 'newest ulg: {} ({} B reported)'.format(
            newest.name, newest.size_b))

        remote = '{}/{}'.format(log_dir, newest.name)
        outdir = make_report_dir(args.report_dir, 'log_transfer')
        local = os.path.join(outdir, newest.name)

        report.info('Downloading {} -> {}'.format(remote, local))
        elapsed, ftp_err = ftp_download(mav, ftp, remote, local, report)
        if ftp_err is not None or elapsed is None:
            report.fail('ftp_download', ftp_err if ftp_err else 'download failed')
            sys.exit(report.finish())

        if not os.path.exists(local):
            report.fail('ftp_download', 'download reported complete but {} is missing'.format(local))
            sys.exit(report.finish())
        size = os.path.getsize(local)
        rate = (size / 1024.0) / elapsed if elapsed > 0 else 0.0
        report.ok('ftp_download', '{} B in {:.2f}s ({:.1f} KB/s)'.format(size, elapsed, rate))

        if size <= MIN_LOG_SIZE:
            report.fail('log_size', 'downloaded file is only {} B (<= {} B)'.format(size, MIN_LOG_SIZE))
            sys.exit(report.finish())
        report.ok('log_size', '{} B'.format(size))

        with open(local, 'rb') as f:
            head = f.read(7)
        if head != ULOG_MAGIC7:
            report.fail('ulog_magic',
                        'header {} does not match ULog magic {}'.format(
                            head.hex(), ULOG_MAGIC7.hex()))
            sys.exit(report.finish())
        report.ok('ulog_magic', 'ULog magic bytes verified')

        # Optional deep parse with pyulog; degrade gracefully if unavailable.
        try:
            from pyulog import ULog
        except ImportError:
            report.info('pyulog not installed; skipping full parse '
                        '(pip3 install --user pyulog for message counts)')
        else:
            try:
                ulog = ULog(local)
                msg_count = len(ulog.data_list)
                report.ok('pyulog_parse', 'parsed {} logged message series'.format(msg_count))
            except Exception as e:  # noqa: BLE001 - a parse failure is a real finding
                report.fail('pyulog_parse', 'pyulog failed to parse the log: {}'.format(e))
                sys.exit(report.finish())
    finally:
        if shell is not None:
            try:
                shell.close()
            except Exception:  # noqa: BLE001 - cleanup only
                pass
        if mav is not None:
            try:
                mav.close()
            except Exception:  # noqa: BLE001 - cleanup only
                pass

    sys.exit(report.finish())


if __name__ == '__main__':
    main()
