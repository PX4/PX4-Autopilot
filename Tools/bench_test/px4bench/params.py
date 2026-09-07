"""Parameter protocol helpers: PX4 int32 union encoding, read/set/echo.

PX4 transports an INT32 parameter as the raw bit pattern of the int placed
into the float param_value field. To send: pack the int as '<i', reinterpret
those 4 bytes as '<f'. To receive: pack the received float as '<f',
reinterpret as '<i'. This mirrors the union trick the firmware uses; the
param float is never a numeric conversion of the int.

Echo handling is hardware-learned: PX4 can emit more than one PARAM_VALUE
per set (handler reply plus the changed-param announcement, times the number
of mavlink instances), so callers must drain stale PARAM_VALUE messages
before a set and then match the echo by expected value, never consume it
positionally.
"""

import re
import struct
import time

from pymavlink import mavutil

MAV_PARAM_TYPE_INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32

SET_ECHO_TIMEOUT_S = 5.0
READ_TIMEOUT_S = 5.0
READ_UNTIL_TIMEOUT_S = 8.0


def int32_to_param_float(value):
    """Reinterpret an int32 bit pattern as the float carried in param_value."""
    return struct.unpack('<f', struct.pack('<i', int(value)))[0]


def param_float_to_int32(value):
    """Reinterpret a param_value float's bit pattern as an int32."""
    return struct.unpack('<i', struct.pack('<f', float(value)))[0]


def param_id_str(raw):
    """Normalize a PARAM_VALUE.param_id (bytes or str) to a plain string."""
    if isinstance(raw, bytes):
        raw = raw.decode('ascii', errors='replace')
    return raw.rstrip('\x00')


def request_param_read(mav, name):
    """Send PARAM_REQUEST_READ by name (param_index = -1)."""
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component,
        name.encode('ascii'), -1)


def read_param(mav, name, timeout=READ_TIMEOUT_S):
    """PARAM_REQUEST_READ then wait for the matching PARAM_VALUE.

    Returns (int32_value, raw_float) or (None, None) on timeout.
    """
    request_param_read(mav, name)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = mav.recv_match(type='PARAM_VALUE', blocking=True,
                           timeout=max(0.1, deadline - time.monotonic()))
        if m is None:
            continue
        if param_id_str(m.param_id) == name:
            return param_float_to_int32(m.param_value), m.param_value
    return None, None


def set_param_int32(mav, name, value):
    """PARAM_SET an int32 using the union encoding."""
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode('ascii'),
        int32_to_param_float(value),
        MAV_PARAM_TYPE_INT32)


def drain_param_values(mav):
    """Discard any queued PARAM_VALUE messages (stale echoes/broadcasts)."""
    while mav.recv_match(type='PARAM_VALUE', blocking=False) is not None:
        pass


def wait_param_echo(mav, name, expected, timeout=SET_ECHO_TIMEOUT_S):
    """Wait for a PARAM_VALUE for name carrying the expected int32 value.

    PX4 can emit more than one PARAM_VALUE per set (handler reply plus the
    changed-param announcement, times the number of mavlink instances), so
    consuming a single message desyncs the harness by one echo forever.
    Returns (matched, seen) where seen is every value observed for name; a
    genuine wrong-echo firmware bug shows up as matched=False with the wrong
    value(s) in seen.
    """
    seen = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = mav.recv_match(type='PARAM_VALUE', blocking=True,
                           timeout=max(0.1, deadline - time.monotonic()))
        if m is None:
            continue
        if param_id_str(m.param_id) != name:
            continue
        value = param_float_to_int32(m.param_value)
        seen.append(value)
        if value == expected:
            return True, seen
    return False, seen


def read_until(mav, name, expected, timeout=READ_UNTIL_TIMEOUT_S):
    """Poll read_param until the board reports the expected int32 value.

    A PARAM_SET propagates through the param system asynchronously, so an
    echo (or even a single read) right after a set can still reflect the
    prior value. Reading back in a loop until the board actually reports the
    new value confirms it is committed to RAM, which closes the gap before a
    save or a dependent check. This trusts the board's own read, not a queued
    echo, so a stale PARAM_VALUE from an earlier set cannot satisfy it.

    Returns (ok, last_seen): ok True once the board reports expected,
    otherwise False with the last value read (None if nothing read back).
    """
    deadline = time.monotonic() + timeout
    last_seen = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False, last_seen
        value, _ = read_param(mav, name, timeout=min(READ_TIMEOUT_S, remaining))
        if value is not None:
            last_seen = value
            if value == expected:
                return True, last_seen
        else:
            # brief pause so a non-responding read does not busy-spin
            time.sleep(0.2)


# param show <name> prints one line:
#   x + l SDLOG_UTC_OFFSET [used,idx] : 777
# three flag columns then the name; the SECOND flag is the save state:
#   '*' unsaved, '+' saved, ' ' unmodified from default
# (src/systemcmds/param/param.cpp:822-823).
_PARAM_SHOW_RE = re.compile(
    r'^\s*(?P<used>[x ])\s?(?P<saved>[*+ ])\s?(?P<ro>[l ])\s+'
    r'(?P<name>[A-Z0-9_]+)\s+\[[^\]]*\]\s*:\s*(?P<value>-?\d+)')


def parse_param_show(output, name):
    """Parse `param show <name>` output for one parameter.

    Returns (saved, value) where saved is True/False/None (None when the
    line is not found or the save state is 'default'/unknown) and value is
    the int32 value or None. Tolerant of ANSI escapes and the prompt.
    """
    for raw in output.splitlines():
        line = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', raw).replace('nsh>', '')
        m = _PARAM_SHOW_RE.search(line)
        if not m or m.group('name') != name:
            continue
        flag = m.group('saved')
        saved = True if flag == '+' else (False if flag == '*' else None)
        try:
            value = int(m.group('value'))
        except ValueError:
            value = None
        return saved, value
    return None, None


def param_is_saved(shell, name, timeout=10):
    """Run `param show <name>` over an open MavlinkShell and report save state.

    Returns (saved, value, timed_out): saved True/False/None as parsed by
    parse_param_show, value the int32 shown, timed_out True if the shell
    command did not complete. Caller owns the shell (open and close it).
    """
    out, timed_out = shell.run('param show {}'.format(name), timeout=timeout)
    if timed_out:
        return None, None, True
    saved, value = parse_param_show(out, name)
    return saved, value, False
