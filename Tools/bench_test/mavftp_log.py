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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from px4bench import Reporter, MavlinkShell, add_connection_args, connect, make_report_dir

from pymavlink import mavutil
from pymavlink import mavftp


LOG_ROOT = '/fs/microsd/log'
# ULog file header magic: 'U' 'L' 'o' 'g' 0x01 0x12 0x35, then a file-version
# byte. See src/modules/logger/logger.cpp:2099-2106.
ULOG_MAGIC7 = bytes([0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35])
FTP_BUDGET = 300.0              # overall MAVFTP budget in seconds
SHELL_TIMEOUT = 10.0
MIN_LOG_SIZE = 1024            # a real ULog is more than 1 KB


def send_heartbeat(mav):
    mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def ftp_list(ftp, path):
    """List a remote directory. cmd_list drives its own reply loop internally
    and populates ftp.list_result. Returns the list of DirectoryEntry.
    """
    ftp.cmd_list([path])
    return list(ftp.list_result)


def ftp_download(mav, ftp, remote, local, report):
    """Download one remote file to a local path over MAVFTP, bounded by
    FTP_BUDGET and reporting a stall by byte count.

    cmd_get only sends OpenFileRO and returns; the transfer itself is driven
    by process_ftp_reply, which loops recv_match -> __mavlink_packet (burst
    read handling, temp-file writes) -> __idle_task until idle or timeout.
    We use a completion callback so we know the burst read finished, and we
    pump process_ftp_reply in bounded slices so a stalled burst becomes a
    named FAIL instead of a hang. process_ftp_reply requires its timeout to
    be greater than idle_detection_time (default 3.7s).

    Returns (elapsed_seconds, None) on success, (None, error_string) on
    failure, so the caller can branch on the error explicitly instead of
    ever mixing a message string into arithmetic.
    """
    done = {'ok': False}

    def on_complete(fh):
        # fh is a BytesIO on success, or None if the session was terminated
        # (failure). cmd_get with a callback keeps the payload in memory, so we
        # persist it here ourselves.
        if fh is None:
            return
        try:
            fh.seek(0)
            with open(local, 'wb') as out:
                out.write(fh.read())
            done['ok'] = True
        except Exception as e:  # noqa: BLE001 - surface write failures to the caller
            report.info('local write failed: {}'.format(e))

    ftp.cmd_get([remote], callback=on_complete)

    start = time.monotonic()
    last_bytes = 0
    last_progress = start
    next_hb = 0.0
    # process_ftp_reply timeout must exceed idle_detection_time; use a short
    # slice and loop so we can enforce our own budget and stall detection.
    slice_timeout = max(4.0, ftp.ftp_settings.idle_detection_time + 0.5)
    while not done['ok']:
        now = time.monotonic()
        if now > next_hb:
            send_heartbeat(mav)
            next_hb = now + 1.0
        if now - start > FTP_BUDGET:
            cur = ftp.read_total if ftp.read_total else last_bytes
            return None, 'MAVFTP transfer stalled at {} bytes (budget {:.0f}s)'.format(
                cur, FTP_BUDGET)
        # drive one slice of the FTP state machine
        ftp.process_ftp_reply('OpenFileRO', timeout=slice_timeout)
        cur = ftp.read_total
        if cur > last_bytes:
            last_bytes = cur
            last_progress = now
        elif now - last_progress > 30.0 and not done['ok']:
            # no forward progress for 30s and callback never fired
            return None, 'MAVFTP transfer stalled at {} bytes (no progress 30s)'.format(cur)
        # if the session ended without completing, process_ftp_reply keeps
        # returning immediately on idle; break out via the budget/stall checks
    elapsed = time.monotonic() - start
    return elapsed, None


def main():
    parser = argparse.ArgumentParser(
        description='Logger + MAVFTP download path test (v1.18 bench).')
    add_connection_args(parser)
    parser.add_argument('--log-duration', type=float, default=10,
                        help='seconds to log before stopping (default: %(default)s)')
    parser.add_argument('--outdir', default='bench_reports',
                        help='base directory for the downloaded log (default: %(default)s)')
    args = parser.parse_args()

    report = Reporter('mavftp_log')

    mav = None
    try:
        report.info('Connecting: {} @ {}'.format(args.connection, args.baudrate))
        try:
            mav = connect(args.connection, baud=args.baudrate, timeout=args.connect_timeout)
        except (TimeoutError, OSError) as e:
            report.fail('connect', str(e))
            sys.exit(report.finish())

        shell = MavlinkShell(mav)
        if not shell.open(timeout=5):
            report.fail('shell_open', 'nsh shell did not respond within 5s')
            sys.exit(report.finish())

        # logger status: confirm the module is running before we tell it to log
        out, timed_out = shell.run('logger status', timeout=SHELL_TIMEOUT)
        if timed_out:
            report.fail('logger_status', '`logger status` stalled (no output in {:.0f}s)'.format(
                SHELL_TIMEOUT))
            shell.close()
            sys.exit(report.finish())
        low = out.lower()
        if ('not running' in low or 'never' in low or
                (low.strip() == '') or ('running' not in low and 'log' not in low)):
            report.fail('logger_status',
                        'logger module not running (check SDLOG_MODE): {}'.format(out.strip()[:200]))
            shell.close()
            sys.exit(report.finish())
        report.ok('logger_status', 'logger module running')

        out, timed_out = shell.run('logger on', timeout=SHELL_TIMEOUT)
        if timed_out:
            report.fail('logger_on', '`logger on` stalled')
            shell.close()
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
            shell.close()
            sys.exit(report.finish())
        report.ok('logger_off', 'logging stopped and flushed')
        shell.close()
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
        outdir = make_report_dir(args.outdir, 'mavftp_log')
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
        if mav is not None:
            try:
                mav.close()
            except Exception:  # noqa: BLE001 - cleanup only
                pass

    sys.exit(report.finish())


if __name__ == '__main__':
    main()
