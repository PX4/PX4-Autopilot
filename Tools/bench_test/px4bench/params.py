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

import struct
import time

from pymavlink import mavutil

MAV_PARAM_TYPE_INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32

SET_ECHO_TIMEOUT_S = 5.0
READ_TIMEOUT_S = 5.0


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
