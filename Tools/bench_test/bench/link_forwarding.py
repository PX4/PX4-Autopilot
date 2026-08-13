#!/usr/bin/env python3
"""
Dual-link forwarding stress test for the mavlink nested-send lock path
(link_forwarding).

This is the flagship bench test for qualifying PX4 v1.18 on real NuttX
hardware after the mavlink locking rework. The rework changed how the
mavlink module takes its send lock, and nested-send paths (a send issued
from within a handler that already holds the lock) were silently
deadlockable on NuttX. A deadlock there does not crash: it hangs. Every
byte over both links just stops.

This test drives two links simultaneously (a USB/telemetry pair) and
hammers the nested-send path hardest in phase 3, where a full parameter
download runs over link2 while an nsh shell session and message pump run
over link1 at the same time. If the send lock can deadlock under nested
use, one of the phases will stop producing traffic and the corresponding
wait will time out. A timeout is a FAIL that names exactly what stalled.

Nothing here arms the vehicle. A hung phase must never hang the script:
every wait is bounded and there is a global wall-clock deadline.
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from px4bench import (Reporter, MavlinkShell, add_connection_args, connect,
                      parse_mavlink_status, send_heartbeat)


HEARTBEAT_INTERVAL = 1.0        # GCS -> autopilot heartbeat cadence, seconds
MAX_SILENT_GAP = 5.0            # a gap larger than this on either link is a FAIL
SHELL_STATUS_INTERVAL = 5.0     # run `mavlink status` over link1 this often in phase 3
SHELL_RUN_TIMEOUT = 10.0        # per shell command timeout in phase 3
PARAM_STALL_TIMEOUT = 10.0      # no new params for this long during download -> stalled
PARAM_HARD_CAP = 120.0          # absolute cap on the param download, seconds
GLOBAL_BUDGET_EXTRA = 300.0     # wall-clock budget = duration + this


class ParamDownloader(threading.Thread):
    """Full parameter download over one link, run in a daemon worker thread.

    Exposes progress (received / expected) so a stalled download can be
    reported by index. The thread stops itself if no new parameter arrives
    within PARAM_STALL_TIMEOUT or the hard cap is hit.
    """

    def __init__(self, mav):
        super().__init__(daemon=True)
        self.mav = mav
        self.received = 0
        self.expected = 0          # param_count reported by the autopilot, 0 until known
        self.complete = False
        self.error = None
        self._seen = set()
        self._last_progress = time.monotonic()

    def run(self):
        try:
            self._download()
        except Exception as e:  # noqa: BLE001 - a worker crash must be reported, not lost
            self.error = repr(e)

    def _download(self):
        self.mav.mav.param_request_list_send(self.mav.target_system,
                                             self.mav.target_component)
        start = time.monotonic()
        self._last_progress = start
        next_hb = 0.0
        while True:
            now = time.monotonic()
            if now > next_hb:
                send_heartbeat(self.mav)
                next_hb = now + HEARTBEAT_INTERVAL
            if now - start > PARAM_HARD_CAP:
                self.error = 'hard cap {:.0f}s hit'.format(PARAM_HARD_CAP)
                return
            if now - self._last_progress > PARAM_STALL_TIMEOUT:
                self.error = 'no new param for {:.0f}s'.format(PARAM_STALL_TIMEOUT)
                return
            m = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.2)
            if m is None:
                continue
            if self.expected == 0 and m.param_count > 0:
                self.expected = m.param_count
            if m.param_index not in self._seen:
                self._seen.add(m.param_index)
                self.received = len(self._seen)
                self._last_progress = now
            if self.expected > 0 and self.received >= self.expected:
                self.complete = True
                return


class LinkPump(threading.Thread):
    """Pump one link in a daemon thread: read everything, track gaps and rate,
    send a heartbeat every HEARTBEAT_INTERVAL. Used for phase 2 where both
    links must stay live at once.
    """

    def __init__(self, mav, name, deadline):
        super().__init__(daemon=True)
        self.mav = mav
        self.name = name
        self.deadline = deadline      # monotonic time to stop at
        self.msgs = 0
        self.bytes = 0
        self.max_gap = 0.0
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        last_msg = time.monotonic()
        next_hb = 0.0
        while not self._stop.is_set() and time.monotonic() < self.deadline:
            now = time.monotonic()
            if now > next_hb:
                send_heartbeat(self.mav)
                next_hb = now + HEARTBEAT_INTERVAL
            m = self.mav.recv_match(blocking=True, timeout=0.1)
            now = time.monotonic()
            if m is None:
                gap = now - last_msg
                if gap > self.max_gap:
                    self.max_gap = gap
                continue
            gap = now - last_msg
            if gap > self.max_gap:
                self.max_gap = gap
            last_msg = now
            self.msgs += 1
            try:
                self.bytes += len(m.get_msgbuf())
            except Exception:  # noqa: BLE001 - length is best-effort accounting only
                pass


def phase1_liveness(report, mav1, mav2, global_deadline):
    """Independent heartbeat check on each link. FAIL names the dead link."""
    report.info('Phase 1: liveness on both links')
    for label, mav in (('link1', mav1), ('link2', mav2)):
        if time.monotonic() > global_deadline:
            report.fail('phase1_budget', 'global deadline hit before checking {}'.format(label))
            return False
        send_heartbeat(mav)
        hb = mav.wait_heartbeat(timeout=5)
        if hb is None:
            report.fail('phase1_heartbeat_{}'.format(label),
                        'no heartbeat on {} within 5s (link dead)'.format(label))
            return False
        report.ok('phase1_heartbeat_{}'.format(label), 'heartbeat received')
    return True


def phase2_sustained(report, mav1, mav2, duration, global_deadline):
    """Sustained bidirectional traffic on both links for `duration` seconds."""
    report.info('Phase 2: sustained bidirectional traffic for {}s'.format(duration))
    deadline = min(time.monotonic() + duration, global_deadline)
    pump1 = LinkPump(mav1, 'link1', deadline)
    pump2 = LinkPump(mav2, 'link2', deadline)
    pump1.start()
    pump2.start()
    # Wait for the pumps to finish their window, with a hard join deadline so a
    # wedged pump thread cannot hang the script.
    join_deadline = deadline + 10.0
    for pump in (pump1, pump2):
        remaining = max(0.1, join_deadline - time.monotonic())
        pump.join(timeout=remaining)
    for pump in (pump1, pump2):
        if pump.is_alive():
            pump.stop()
            report.fail('phase2_pump_{}'.format(pump.name),
                        'pump thread on {} did not stop (stalled)'.format(pump.name))
            return False

    elapsed = duration
    for pump in (pump1, pump2):
        rate_msgs = pump.msgs / elapsed if elapsed > 0 else 0.0
        rate_bytes = pump.bytes / elapsed if elapsed > 0 else 0.0
        detail = '{} msgs ({:.1f} msg/s), {} B ({:.1f} B/s), max gap {:.2f}s'.format(
            pump.msgs, rate_msgs, pump.bytes, rate_bytes, pump.max_gap)
        if pump.msgs == 0:
            report.fail('phase2_traffic_{}'.format(pump.name),
                        '{}: zero messages received'.format(pump.name))
            return False
        if pump.max_gap > MAX_SILENT_GAP:
            report.fail('phase2_gap_{}'.format(pump.name),
                        '{}: silent gap {:.2f}s exceeds {:.0f}s: {}'.format(
                            pump.name, pump.max_gap, MAX_SILENT_GAP, detail))
            return False
        report.ok('phase2_traffic_{}'.format(pump.name), detail)
    return True


def phase3_nested_hammer(report, mav1, mav2, global_deadline):
    """Param download over link2 in a worker thread WHILE link1 runs an nsh
    shell (`mavlink status` every 5s) and keeps pumping. This is the nested
    send lock stress: heavy simultaneous use of both links.
    """
    report.info('Phase 3: nested-send hammer (param download on link2, shell on link1)')
    shell = MavlinkShell(mav1)
    if not shell.open(timeout=5):
        report.fail('phase3_shell_open', 'nsh shell over link1 did not respond within 5s')
        return False

    downloader = ParamDownloader(mav2)
    downloader.start()

    hard_deadline = min(time.monotonic() + PARAM_HARD_CAP + 15.0, global_deadline)
    next_status = 0.0
    shell_runs = 0
    try:
        while downloader.is_alive():
            now = time.monotonic()
            if now > hard_deadline:
                break
            # keep link1 pumping so the autopilot keeps streaming to us
            mav1.recv_match(blocking=True, timeout=0.1)
            if now >= next_status:
                _, timed_out = shell.run('mavlink status', timeout=SHELL_RUN_TIMEOUT)
                if timed_out:
                    report.fail('phase3_shell_stall',
                                'shell over link1 stalled during param download on link2 '
                                '(param {}/{})'.format(downloader.received, downloader.expected))
                    return False
                shell_runs += 1
                next_status = now + SHELL_STATUS_INTERVAL

        # Join the worker with a hard deadline. A thread stuck past the deadline
        # is itself the finding; it is a daemon so the script can still exit.
        downloader.join(timeout=max(0.1, hard_deadline - time.monotonic()) + 2.0)
    finally:
        shell.close()

    if downloader.is_alive():
        report.fail('phase3_param_stall',
                    'param download on link2 stalled at index {}/{}'.format(
                        downloader.received, downloader.expected))
        return False
    if downloader.error is not None:
        report.fail('phase3_param_download',
                    'param download on link2 incomplete ({}) at {}/{}'.format(
                        downloader.error, downloader.received, downloader.expected))
        return False
    if not downloader.complete:
        report.fail('phase3_param_download',
                    'param download on link2 incomplete at {}/{}'.format(
                        downloader.received, downloader.expected))
        return False

    report.ok('phase3_param_download',
              'downloaded {}/{} params while running {} shell status calls on link1'.format(
                  downloader.received, downloader.expected, shell_runs))
    report.ok('phase3_shell', '{} `mavlink status` calls completed over link1'.format(shell_runs))
    return True


def phase4_post_liveness(report, mav1, mav2, global_deadline):
    """After the hammer, both links must still deliver a fresh heartbeat and
    link1 must complete one more `mavlink status`. This proves both links
    survived heavy simultaneous use.
    """
    report.info('Phase 4: post-stress liveness')
    for label, mav in (('link1', mav1), ('link2', mav2)):
        if time.monotonic() > global_deadline:
            report.fail('phase4_budget', 'global deadline hit before checking {}'.format(label))
            return False
        send_heartbeat(mav)
        hb = mav.wait_heartbeat(timeout=5)
        if hb is None:
            report.fail('phase4_heartbeat_{}'.format(label),
                        'no fresh heartbeat on {} within 5s after stress'.format(label))
            return False
        report.ok('phase4_heartbeat_{}'.format(label), 'link alive after stress')

    shell = MavlinkShell(mav1)
    if not shell.open(timeout=5):
        report.fail('phase4_shell_open', 'nsh shell over link1 did not respond after stress')
        return False
    try:
        out, timed_out = shell.run('mavlink status', timeout=SHELL_RUN_TIMEOUT)
    finally:
        shell.close()
    if timed_out:
        report.fail('phase4_shell_stall', '`mavlink status` over link1 stalled after stress')
        return False
    instances = parse_mavlink_status(out)
    if not instances:
        report.fail('phase4_status_parse', 'could not parse any instance block from mavlink status')
        return False
    for inst in instances:
        report.info('  {} | {} | {}'.format(
            inst['header'],
            inst['tx'] if inst['tx'] else 'tx: ?',
            inst['rx'] if inst['rx'] else 'rx: ?'))
    report.ok('phase4_status', 'parsed {} mavlink instance block(s)'.format(len(instances)))
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Dual-link forwarding stress of the mavlink nested-send lock path.')
    add_connection_args(parser, dual_link=True)
    parser.add_argument('--baudrate2', type=int, default=57600,
                        help='baud rate for the second connection (default: %(default)s)')
    parser.add_argument('--duration', type=float, default=60,
                        help='sustained-traffic duration in seconds (default: %(default)s)')
    args = parser.parse_args()

    if args.connection2 is None:
        print('error: link_forwarding requires a second connection (CONNECTION2)',
              file=sys.stderr)
        sys.exit(2)

    report = Reporter('link_forwarding')
    global_deadline = time.monotonic() + args.duration + GLOBAL_BUDGET_EXTRA

    mav1 = None
    mav2 = None
    try:
        report.info('Connecting link1: {} @ {}'.format(args.connection, args.baudrate))
        try:
            mav1 = connect(args.connection, baud=args.baudrate, timeout=args.connect_timeout)
        except (TimeoutError, OSError) as e:
            report.fail('connect_link1', str(e))
            sys.exit(report.finish())

        report.info('Connecting link2: {} @ {}'.format(args.connection2, args.baudrate2))
        try:
            mav2 = connect(args.connection2, baud=args.baudrate2, timeout=args.connect_timeout)
        except (TimeoutError, OSError) as e:
            report.fail('connect_link2', str(e))
            sys.exit(report.finish())

        if not phase1_liveness(report, mav1, mav2, global_deadline):
            sys.exit(report.finish())
        if not phase2_sustained(report, mav1, mav2, args.duration, global_deadline):
            sys.exit(report.finish())
        if not phase3_nested_hammer(report, mav1, mav2, global_deadline):
            sys.exit(report.finish())
        if not phase4_post_liveness(report, mav1, mav2, global_deadline):
            sys.exit(report.finish())
    finally:
        for mav in (mav1, mav2):
            if mav is not None:
                try:
                    mav.close()
                except Exception:  # noqa: BLE001 - cleanup only
                    pass

    sys.exit(report.finish())


if __name__ == '__main__':
    main()
