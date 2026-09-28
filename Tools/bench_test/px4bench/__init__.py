"""px4bench: shared library for the PX4 bench-test suite (Tools/bench_test).

Core primitives every test builds on: MAVLink connection setup, PASS/FAIL
reporting, a non-interactive nsh shell over SERIAL_CONTROL (pattern from
Tools/mavlink_shell.py), reboot and USB re-detection helpers, and small
parsers for `mavlink status` output.

Protocol-area helpers live in submodules:
    px4bench.params    parameter read/set/echo with PX4's int32 union encoding
    px4bench.missions  mission item generation and the upload/download protocol
    px4bench.ftp       MAVFTP listing/download and ULog constants

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

# pymavlink 2.4.49: add_message() crashes with TypeError if the first message
# of an instanced type (BATTERY_STATUS, ESC_STATUS, ...) arrives with its
# instance field unset -- it is then stored without the _instances dict and the
# next instanced arrival does None[key]. Timing-dependent, so it bites
# intermittently mid-suite. Wrap it; drop once fixed upstream.
_orig_add_message = mavutil.add_message


def _safe_add_message(messages, mtype, msg):
    try:
        _orig_add_message(messages, mtype, msg)
    except TypeError:
        messages[mtype] = msg


mavutil.add_message = _safe_add_message

SERIAL_CONTROL_DEV_SHELL = 10  # SERIAL_CONTROL_DEV_SHELL (see Tools/mavlink_shell.py)
DEFAULT_BAUD = 57600
USB_DEVICE_GLOB_DARWIN = '/dev/tty.usbmodem*'
USB_DEVICE_GLOB_LINUX = '/dev/serial/by-id/*PX4*'

# Exit code meaning "test skipped" (EX_TEMPFAIL): the orchestrator records it
# as SKIP, not FAIL. Used when a probe finds the firmware lacks a needed
# command, no SD card is mounted, or the operator declines the arming gate.
EXIT_SKIP = 75

# NuttX nsh prints exactly this for an unknown command
# (apps/nshlib/nsh_parse.c: "nsh: %s: command not found").
NSH_NOT_FOUND = 'command not found'


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


FIRMWARE_INFO_ENV = 'PX4BENCH_FIRMWARE_INFO'


def make_report_dir(base='bench_reports', test_name=''):
    """Create and return a timestamped report directory.

    When the suite preflight has established the firmware identity it
    exports it via PX4BENCH_FIRMWARE_INFO (a JSON string); stamp it into
    every report dir so each test result is traceable to a build.
    """
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = os.path.join(base, '{}_{}'.format(stamp, test_name) if test_name else stamp)
    os.makedirs(path, exist_ok=True)
    info = os.environ.get(FIRMWARE_INFO_ENV)
    if info:
        try:
            with open(os.path.join(path, 'firmware.json'), 'w') as f:
                f.write(info.rstrip('\n') + '\n')
        except OSError:
            pass
    return path


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
    send_heartbeat(mav)
    hb = mav.wait_heartbeat(timeout=int(timeout))
    if hb is None:
        mav.close()
        raise TimeoutError('no HEARTBEAT on {} within {}s'.format(conn_str, timeout))
    return mav


def wait_heartbeat(mav, timeout: float = 10):
    """Wait for the next autopilot heartbeat. Returns the message or None."""
    return mav.wait_heartbeat(timeout=int(timeout))


def send_heartbeat(mav):
    """Send one GCS heartbeat so the autopilot keeps streaming to us."""
    mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def drain(mav, duration=0.5):
    """Read and discard pending messages for a short window."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if mav.recv_match(blocking=True, timeout=0.05) is None:
            pass


def parse_mavlink_status(text):
    """Parse `mavlink status` shell output into per-instance dicts.

    Each instance block begins with `instance #N:` and contains rate lines
    `tx: X B/s` / `rx: X B/s` (src/modules/mavlink/mavlink_main.cpp). Returns
    a list of {'header', 'tx', 'rx'} dicts; its length is the instance count.
    """
    instances = []
    cur = None
    for line in text.splitlines():
        stripped = line.strip()
        if 'instance #' in stripped:
            if cur is not None:
                instances.append(cur)
            cur = {'header': stripped, 'tx': None, 'rx': None}
            continue
        if cur is None:
            continue
        if stripped.startswith('tx:') and cur['tx'] is None:
            cur['tx'] = stripped
        elif stripped.startswith('rx:') and cur['rx'] is None:
            cur['rx'] = stripped
    if cur is not None:
        instances.append(cur)
    return instances


def count_mavlink_instances(text):
    """Count MAVLink instances in `mavlink status` output."""
    return len(re.findall(r'instance\s*#\s*\d+', text))


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
            send_heartbeat(self.mav)
            self._next_heartbeat = now + 1.0
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True, timeout=window)
        if m is not None:
            self.buf += ''.join(chr(x) for x in m.data[:m.count])

    def open(self, timeout: float = 5):
        """Wake the shell and confirm a live prompt. Returns True on success.

        The firmware spawns a fresh nsh task plus two pipes on the first
        SERIAL_CONTROL write of a session, so opening is not free; we must
        confirm the task is actually up and reading, not just that some byte
        arrived. Leftover bytes from a prior session would pass a naive
        len(buf) > 0 check and report a shell that is not there.

        We require either the 'nsh>' prompt token, or a completed sentinel
        round-trip driven through the shell: sending 'echo <sentinel>' and
        seeing that sentinel echoed back proves nsh is up and processing our
        input. The sentinel round-trip also covers builds whose prompt token
        differs or is suppressed.
        """
        self._seq += 1
        sentinel = 'BENCHOPEN{}'.format(self._seq)
        self.buf = ''
        self._write('\n')
        self._write('echo {}\n'.format(sentinel))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump()
            if 'nsh>' in self.buf or re.search(
                    r'^{}\s*$'.format(re.escape(sentinel)), self.buf, re.MULTILINE):
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
        # The sentinel echoes go on their OWN line: nsh aborts the rest of a
        # ';' chain when a command fails (including 'command not found'), so
        # chaining them onto the command line loses the sentinel for any
        # non-zero exit and a failing command reads as a stall. nsh consumes
        # input lines sequentially, so the sentinel line runs after the
        # command finishes regardless of its exit status.
        # Two echoes like run_nsh_cmd.py, in case the first line is garbled.
        self._write('{}\n'.format(cmd))
        self._write('echo {0}; echo {0}\n'.format(sentinel))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump()
            # sentinel on its own line (not the command echo containing "echo <sentinel>")
            if re.search(r'^{}\s*$'.format(re.escape(sentinel)), self.buf, re.MULTILINE):
                return self._strip(cmd, sentinel), False
        return self._strip(cmd, sentinel), True

    def _strip(self, cmd, sentinel):
        """Remove command echo and sentinel lines from captured output.

        Strips ANY BENCHDONE sentinel, not just the current one: the second
        safety echo of the previous command often arrives after run() already
        returned and would otherwise leak into this command's output.
        """
        lines = []
        for line in self.buf.splitlines():
            if sentinel in line or re.match(r'BENCHDONE\d+\s*$', line.strip()):
                continue
            if cmd in line and 'echo' in line:
                continue
            # the terminal echo of the command itself (possibly prefixed by
            # the prompt and ANSI erase sequences) is not command output
            bare = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', line).replace('nsh>', '').strip()
            if bare == cmd:
                continue
            lines.append(line)
        return '\n'.join(lines).strip()

    def close(self, drain: float = 1.0):
        """Tear down the firmware shell session and confirm teardown.

        A SERIAL_CONTROL message with the RESPOND flag cleared (flags=0) is
        what triggers Mavlink::close_shell() and frees the nsh task and its
        two pipes. Sending it and returning immediately lets a caller reopen
        before the firmware has processed the teardown, which spawns a second
        shell before the first is freed: repeat that and the board's task/fd
        table is exhausted (a leak we drive, not a firmware hang). So after
        sending flags=0 we briefly drain incoming SERIAL_CONTROL traffic to
        give the firmware time to run close_shell before the caller can
        reopen. Bounded wait, never a hang.
        """
        try:
            self.mav.mav.serial_control_send(SERIAL_CONTROL_DEV_SHELL, 0, 0, 0, 0, [0] * 70)
        except Exception:
            return
        deadline = time.monotonic() + drain
        while time.monotonic() < deadline:
            m = self.mav.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.1)
            if m is None:
                # a quiet window means the shell has stopped emitting; teardown
                # has been processed
                break


def shell_command_exists(shell, probe_cmd, timeout=10):
    """Probe the live firmware for a command via an open MavlinkShell.

    probe_cmd should be a harmless invocation (e.g. 'simulator_sih status',
    'sd_bench -h'): only the not-found reply matters, not the exit status.
    Returns (present, output):
      (True, out)  the command exists in this firmware
      (False, out) nsh replied 'command not found'
      (None, out)  the probe itself stalled, which is its own finding
    """
    out, timed_out = shell.run(probe_cmd, timeout=timeout)
    if timed_out:
        return None, out
    if NSH_NOT_FOUND in out:
        return False, out
    return True, out


def arming_gate(allow_arming, action='run the SIH flight test'):
    """Confirm that arming the (simulated) vehicle is acceptable.

    The flight test arms the flight controller. Physics is simulated on the
    FMU and pwm_out_sim replaces the real output drivers, but the board must
    be bare: nothing may be connected to the output rails.

    Returns True to proceed, False to skip:
      --allow-arming given        -> proceed without prompting (automation)
      interactive TTY             -> proceed only if the operator types 'arm'
      non-TTY without the flag    -> skip, naming --allow-arming
    """
    if allow_arming:
        return True
    if not sys.stdin.isatty():
        print('WARNING: this test arms the flight controller (simulated '
              'flight). Non-interactive runs must pass --allow-arming to '
              'confirm the board is bare; skipping.', flush=True)
        return False
    print('!' * 70)
    print('WARNING: about to {}.'.format(action))
    print('The flight controller WILL ARM. The flight is simulated on the')
    print('FMU and pwm_out_sim replaces the real output drivers, but the')
    print('board must be bare: NOTHING may be connected to the output rails')
    print('(no ESCs, no motors, no servos).')
    print('!' * 70)
    answer = input("Type 'arm' to continue, anything else to skip: ").strip().lower()
    if answer == 'arm':
        return True
    print('operator declined; skipping.', flush=True)
    return False


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


def wait_reconnect(conn_str, baud=DEFAULT_BAUD, timeout=60, start=None):
    """Wait for a rebooted/re-enumerated board and reconnect.

    For a serial device: waits for the node to vanish and return (the node
    name may change across enumeration), lets the CDC ACM interface settle,
    then retries connect() until a heartbeat arrives. Returns
    (new_mav_connection, elapsed_seconds). Raises TimeoutError with a
    description of what stalled.
    """
    if start is None:
        start = time.monotonic()

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


def reboot_and_reconnect(mav, conn_str, baud=DEFAULT_BAUD, timeout=60):
    """Reboot the autopilot and reconnect.

    Returns (new_mav_connection, elapsed_seconds).
    Raises TimeoutError with a description of what stalled.
    """
    start = time.monotonic()
    send_reboot(mav)
    time.sleep(0.5)
    mav.close()
    return wait_reconnect(conn_str, baud=baud, timeout=timeout, start=start)


def add_connection_args(parser, dual_link=False):
    """Standard CLI surface shared by every test in the suite."""
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
