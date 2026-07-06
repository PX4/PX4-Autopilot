#!/usr/bin/env python3
"""
Shared helpers for the PX4 bench-test suite (Tools/bench_test).

Provides MAVLink connection setup, a non-interactive nsh shell over
SERIAL_CONTROL (reusing the pattern from Tools/mavlink_shell.py), reboot
and USB re-detection helpers, and PASS/FAIL reporting.

Every operation takes a timeout: a hung board is a finding, not an excuse
for a hung test script.
"""

import glob
import os
import re
import sys
import time
from datetime import datetime, timezone

PYMAVLINK_INSTALL_HINT = """\
Failed to import pymavlink: {err}

Install it with:
    pip3 install --user pymavlink
"""

try:
    from pymavlink import mavutil
except ImportError as e:
    print(PYMAVLINK_INSTALL_HINT.format(err=e))
    sys.exit(2)

SERIAL_CONTROL_DEV_SHELL = 10  # SERIAL_CONTROL_DEV_SHELL (see Tools/mavlink_shell.py)
DEFAULT_BAUD = 57600
USB_DEVICE_GLOB_DARWIN = '/dev/tty.usbmodem*'
USB_DEVICE_GLOB_LINUX = '/dev/serial/by-id/*PX4*'


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

class Reporter:
    """Collects named PASS/FAIL checks and prints a summary.

    Usage:
        report = Reporter('boot_health')
        report.check('heartbeat', ok, 'detail string')
        ...
        sys.exit(report.finish())
    """

    def __init__(self, test_name):
        self.test_name = test_name
        self.results = []  # (name, ok, detail)
        self.start_time = time.monotonic()

    def check(self, name, ok, detail=''):
        tag = 'PASS' if ok else 'FAIL'
        msg = '[{}] {}: {}'.format(tag, name, detail) if detail else '[{}] {}'.format(tag, name)
        print(msg, flush=True)
        self.results.append((name, bool(ok), detail))
        return bool(ok)

    def fail(self, name, detail=''):
        return self.check(name, False, detail)

    def ok(self, name, detail=''):
        return self.check(name, True, detail)

    def info(self, msg):
        print('[INFO] {}'.format(msg), flush=True)

    @property
    def num_failed(self):
        return sum(1 for _, ok, _ in self.results if not ok)

    def finish(self):
        """Print summary, return process exit code (0 = all passed)."""
        elapsed = time.monotonic() - self.start_time
        total = len(self.results)
        failed = self.num_failed
        print('-' * 60)
        if failed == 0:
            print('{}: PASS ({} checks, {:.1f}s)'.format(self.test_name, total, elapsed))
        else:
            print('{}: FAIL ({}/{} checks failed, {:.1f}s)'.format(
                self.test_name, failed, total, elapsed))
            for name, ok, detail in self.results:
                if not ok:
                    print('  FAILED: {}{}'.format(name, ' - ' + detail if detail else ''))
        return 0 if failed == 0 else 1


def make_report_dir(base='bench_reports', test_name=''):
    """Create and return a timestamped report directory."""
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = os.path.join(base, '{}_{}'.format(stamp, test_name) if test_name else stamp)
    os.makedirs(path, exist_ok=True)
    return path


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

def is_serial_device(conn_str):
    return conn_str.startswith('/dev/') or conn_str.upper().startswith('COM')


def connect(conn_str, baud=DEFAULT_BAUD, timeout: float = 20, source_system=254):
    """Open a MAVLink connection and wait for a heartbeat from the autopilot.

    Returns the mavutil connection, or raises TimeoutError/OSError.
    """
    mav = mavutil.mavlink_connection(conn_str, baud=baud, autoreconnect=False,
                                     source_system=source_system,
                                     source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
    assert isinstance(mav, mavutil.mavfile), 'unexpected connection type for {}'.format(conn_str)
    # announce ourselves so the autopilot streams to us
    mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb = mav.wait_heartbeat(timeout=int(timeout))
    if hb is None:
        mav.close()
        raise TimeoutError('no HEARTBEAT on {} within {}s'.format(conn_str, timeout))
    return mav


def wait_heartbeat(mav, timeout: float = 10):
    """Wait for the next autopilot heartbeat. Returns the message or None."""
    return mav.wait_heartbeat(timeout=int(timeout))


def drain(mav, duration=0.5):
    """Read and discard pending messages for a short window."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if mav.recv_match(blocking=True, timeout=0.05) is None:
            pass


# ---------------------------------------------------------------------------
# nsh shell over MAVLink SERIAL_CONTROL
# ---------------------------------------------------------------------------

class MavlinkShell:
    """Non-interactive nsh shell over SERIAL_CONTROL (dev 10).

    Write pattern follows Tools/mavlink_shell.py; completion detection uses
    the sentinel-echo trick from Tools/HIL/run_nsh_cmd.py:
        <cmd>; echo <sentinel>
    so we know when output for <cmd> is complete without relying on prompt
    parsing.
    """

    def __init__(self, mav):
        self.mav = mav
        self.buf = ''
        self._next_heartbeat = 0.0
        self._seq = 0

    def _write(self, text):
        b = text
        while len(b) > 0:
            n = min(len(b), 70)
            buf = [ord(c) for c in b[:n]]
            buf.extend([0] * (70 - len(buf)))
            self.mav.mav.serial_control_send(
                SERIAL_CONTROL_DEV_SHELL,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0, 0, n, buf)
            b = b[n:]

    def _pump(self, window=0.03):
        """Read pending SERIAL_CONTROL data into self.buf, keep heartbeats going."""
        now = time.monotonic()
        if now > self._next_heartbeat:
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._next_heartbeat = now + 1.0
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True, timeout=window)
        if m is not None:
            self.buf += ''.join(chr(x) for x in m.data[:m.count])

    def open(self, timeout: float = 5):
        """Wake the shell; returns True if any shell output is seen."""
        self._write('\n')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump()
            if 'nsh>' in self.buf or len(self.buf) > 0:
                self.buf = ''
                return True
        return False

    def run(self, cmd, timeout: float = 10):
        """Run one command, return (output_str, timed_out_bool).

        On timeout the partial output is returned and timed_out is True --
        a command that never completes is exactly the hang we are hunting.
        """
        self._seq += 1
        sentinel = 'BENCHDONE{}'.format(self._seq)
        self.buf = ''
        # two echos like run_nsh_cmd.py, in case the first line is garbled
        self._write('{}; echo {}; echo {}\n'.format(cmd, sentinel, sentinel))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump()
            # sentinel on its own line (not the command echo containing "echo <sentinel>")
            if re.search(r'^{}\s*$'.format(re.escape(sentinel)), self.buf, re.MULTILINE):
                return self._strip(cmd, sentinel), False
        return self._strip(cmd, sentinel), True

    def _strip(self, cmd, sentinel):
        """Remove command echo and sentinel lines from captured output."""
        lines = []
        for line in self.buf.splitlines():
            if sentinel in line:
                continue
            if cmd in line and 'echo' in line:
                continue
            lines.append(line)
        return '\n'.join(lines).strip()

    def close(self):
        try:
            self.mav.mav.serial_control_send(SERIAL_CONTROL_DEV_SHELL, 0, 0, 0, 0, [0] * 70)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Reboot / device replug helpers
# ---------------------------------------------------------------------------

def send_reboot(mav):
    """Request an autopilot reboot (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1)."""
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0)


def list_usb_devices(pattern=None):
    """List candidate PX4 USB serial devices."""
    if pattern:
        return sorted(glob.glob(pattern))
    devs = sorted(glob.glob(USB_DEVICE_GLOB_DARWIN))
    if not devs:
        devs = sorted(glob.glob(USB_DEVICE_GLOB_LINUX))
        if not devs:
            devs = sorted(glob.glob('/dev/ttyACM*'))
    return devs


def wait_device_gone(device, timeout=30):
    """Wait until the serial device node disappears. Returns elapsed or None."""
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        if not os.path.exists(device):
            return time.monotonic() - start
        time.sleep(0.2)
    return None


def wait_device_back(device, timeout=45, pattern=None):
    """Wait until the device node (or a matching one) reappears.

    USB enumeration can change the node name (usbmodem01 -> usbmodem101 on
    macOS), so if the exact node does not come back we fall back to a glob
    for the same device family. Returns (device_path, elapsed) or (None, None).
    """
    if pattern is None:
        base = re.sub(r'\d+$', '', device)
        pattern = base + '*' if base != device else None
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        if os.path.exists(device):
            return device, time.monotonic() - start
        if pattern:
            found = sorted(glob.glob(pattern))
            if found:
                return found[0], time.monotonic() - start
        time.sleep(0.2)
    return None, None


def reboot_and_reconnect(mav, conn_str, baud=DEFAULT_BAUD, timeout=60):
    """Reboot the autopilot and reconnect.

    Returns (new_mav_connection, elapsed_seconds).
    Raises TimeoutError with a description of what stalled.
    """
    start = time.monotonic()
    send_reboot(mav)
    time.sleep(0.5)
    mav.close()

    if is_serial_device(conn_str):
        gone = wait_device_gone(conn_str, timeout=15)
        # Some boards re-enumerate too fast to observe the node vanish; that is fine.
        newdev, _ = wait_device_back(conn_str, timeout=timeout)
        if newdev is None:
            raise TimeoutError('serial device {} did not come back within {}s '
                               '(gone after {})'.format(
                                   conn_str, timeout,
                                   '{:.1f}s'.format(gone) if gone is not None else 'n/a'))
        conn_str = newdev
        time.sleep(2.0)  # let the CDC ACM interface settle
    else:
        time.sleep(3.0)

    remaining = max(5.0, timeout - (time.monotonic() - start))
    last_err = None
    while time.monotonic() - start < timeout:
        try:
            newmav = connect(conn_str, baud=baud, timeout=int(max(1, min(10, remaining))))
            return newmav, time.monotonic() - start
        except (TimeoutError, OSError) as e:
            last_err = e
            time.sleep(1.0)
    raise TimeoutError('no heartbeat after reboot within {}s (last error: {})'.format(
        timeout, last_err))


# ---------------------------------------------------------------------------
# Common CLI plumbing
# ---------------------------------------------------------------------------

def add_connection_args(parser, dual_link=False):
    parser.add_argument('connection',
                        help='MAVLink connection: serial device (/dev/tty.usbmodem01), '
                             'udp:IP:PORT, or tcp:IP:PORT')
    if dual_link:
        parser.add_argument('connection2', nargs='?', default=None,
                            help='optional second MAVLink connection (telemetry radio)')
    parser.add_argument('--baudrate', '-b', type=int, default=DEFAULT_BAUD,
                        help='serial baud rate (default: %(default)s)')
    parser.add_argument('--connect-timeout', type=float, default=20,
                        help='seconds to wait for the first heartbeat (default: %(default)s)')
