#!/usr/bin/env python3
"""
MAVLink parameter-validation regression test.

Verifies that PX4 rejects mission items and commands carrying non-zero
values in parameter slots that the MAVLink spec marks as unsupported for
that MAV_CMD, and accepts identical items/commands with those slots
zeroed or NaN.

Usage:
    python3 test_mavlink_param_validation.py [--url udp://:14540] [--timeout 5]

Requirements:
    pip install pymavlink
"""

import argparse
import sys
import time
from typing import Any, Optional

try:
    from pymavlink import mavutil  # type: ignore[import-not-found]
except ImportError:
    print("ERROR: pymavlink not installed.  Run: pip install pymavlink")
    sys.exit(1)

# MAVLink enum constants

MAV_MISSION_ACCEPTED = 0
MAV_MISSION_INVALID_PARAM1 = 6
MAV_MISSION_INVALID_PARAM2 = 7
MAV_MISSION_INVALID_PARAM3 = 8
MAV_MISSION_INVALID_PARAM4 = 9
MAV_MISSION_INVALID_PARAM5 = 10
MAV_MISSION_INVALID_PARAM6 = 11
MAV_MISSION_INVALID_PARAM7 = 12

MAV_RESULT_ACCEPTED = 0
MAV_RESULT_DENIED = 2

MAV_FRAME_GLOBAL_INT = 5
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_FRAME_MISSION = 2

# MAV_CMD values used in tests
CMD_NAV_WAYPOINT = 16
CMD_NAV_RTL = 20
CMD_NAV_TAKEOFF = 22
CMD_NAV_DELAY = 93
CMD_COMPONENT_ARM_DISARM = 400

NAN = float("nan")
INT32_MAX = 2_147_483_647

# Helpers

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
_results: list[bool] = []


def _check(label: str, got: Any, expected: Any) -> bool:
    ok = bool(got == expected)
    _results.append(ok)
    status = PASS if ok else FAIL
    exp_name = (
        _mission_result_name(expected)
        if isinstance(expected, int) else expected
    )
    got_name = (
        _mission_result_name(got)
        if isinstance(got, int) else got
    )
    print(f"  [{status}] {label}")
    if not ok:
        print(
            f"          got={got_name} ({got}),"
            f" expected={exp_name} ({expected})"
        )
    return ok


def _mission_result_name(v: int) -> str:
    names = {
        0: "ACCEPTED", 6: "INVALID_PARAM1", 7: "INVALID_PARAM2",
        8: "INVALID_PARAM3", 9: "INVALID_PARAM4", 10: "INVALID_PARAM5",
        11: "INVALID_PARAM6", 12: "INVALID_PARAM7",
    }
    return names.get(v, str(v))


def connect(url: str) -> Any:
    mav = mavutil.mavlink_connection(url)
    print(f"Waiting for heartbeat on {url} ...")
    mav.wait_heartbeat(timeout=15)
    print(
        f"Connected: sysid={mav.target_system}"
        f" compid={mav.target_component}\n"
    )
    return mav


def _upload_mission(
    mav: Any, items: list[dict[str, Any]], timeout: float
) -> Optional[int]:
    """
    Run the MISSION_ITEM_INT upload protocol and return the
    MAV_MISSION_RESULT from the final MISSION_ACK, or None on timeout.
    """
    mav.mav.mission_count_send(
        mav.target_system, mav.target_component,
        len(items), 0,  # mission_type=0 (main)
    )

    deadline = time.monotonic() + timeout * (len(items) + 2)

    while time.monotonic() < deadline:
        msg = mav.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
            blocking=True,
            timeout=timeout,
        )
        if msg is None:
            return None

        t = msg.get_type()

        if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            item = items[msg.seq]
            mav.mav.mission_item_int_send(
                item["sys"], item["comp"],
                item["seq"], item["frame"], item["cmd"],
                item["current"], item["autocontinue"],
                item["p1"], item["p2"], item["p3"], item["p4"],
                item["x"], item["y"], item["z"],
                0,  # mission_type
            )

        elif t == "MISSION_ACK":
            return int(msg.type)

    return None


def _item(
    seq: int, cmd: int, frame: int,
    p1: float = NAN, p2: float = NAN,
    p3: float = NAN, p4: float = NAN,
    x: int = INT32_MAX, y: int = INT32_MAX, z: float = 0.0,
    current: int = 0,
) -> dict[str, Any]:
    return dict(
        sys=1, comp=1, seq=seq, cmd=cmd, frame=frame,
        current=current, autocontinue=1,
        p1=p1, p2=p2, p3=p3, p4=p4,
        x=x, y=y, z=z,
    )


def _send_command(
    mav: Any, cmd: int, timeout: float,
    p1: float = 0.0, p2: float = 0.0,
    p3: float = 0.0, p4: float = 0.0,
    p5: float = 0.0, p6: float = 0.0, p7: float = 0.0,
) -> Optional[int]:
    """Send COMMAND_LONG and return the MAV_RESULT from the ACK, or None."""
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        cmd, 0,
        p1, p2, p3, p4, p5, p6, p7,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=timeout,
        )
        if msg and msg.command == cmd:
            return int(msg.result)
    return None


# Test cases


def run_mission_tests(mav: Any, timeout: float) -> None:
    print("=== Mission upload tests ===")

    # Coordinates for a valid waypoint (47.397 N  8.545 E  50 m)
    LAT = 473_977_420  # 1e-7 deg
    LON = 85_456_060   # 1e-7 deg
    ALT = 50.0

    # 1. Valid NAV_WAYPOINT
    # mask 0x0B: p1 (hold), p2 (accept_radius), p4 (yaw); p3 unsupported.
    result = _upload_mission(mav, [
        _item(0, CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_INT,
              p1=0.0, p2=2.0, p3=NAN, p4=NAN,
              x=LAT, y=LON, z=ALT, current=1),
    ], timeout)
    _check("Valid NAV_WAYPOINT -> ACCEPTED", result, MAV_MISSION_ACCEPTED)

    # 2. NAV_WAYPOINT with unsupported param3 set
    result = _upload_mission(mav, [
        _item(0, CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_INT,
              p1=0.0, p2=2.0, p3=1.0, p4=NAN,
              x=LAT, y=LON, z=ALT, current=1),
    ], timeout)
    _check(
        "NAV_WAYPOINT unsupported param3 -> INVALID_PARAM3",
        result, MAV_MISSION_INVALID_PARAM3,
    )

    # 3. NAV_RTL with param1 set (mask 0x00, no params)
    result = _upload_mission(mav, [
        _item(0, CMD_NAV_RTL, MAV_FRAME_GLOBAL_INT,
              p1=1.0, p2=NAN, p3=NAN, p4=NAN,
              x=LAT, y=LON, z=ALT, current=1),
    ], timeout)
    _check(
        "NAV_RTL unsupported param1 -> INVALID_PARAM1",
        result, MAV_MISSION_INVALID_PARAM1,
    )

    # 4. NAV_RTL with all params NaN (should pass)
    result = _upload_mission(mav, [
        _item(0, CMD_NAV_RTL, MAV_FRAME_GLOBAL_INT,
              p1=NAN, p2=NAN, p3=NAN, p4=NAN,
              x=LAT, y=LON, z=ALT, current=1),
    ], timeout)
    _check(
        "NAV_RTL all params NaN -> ACCEPTED",
        result, MAV_MISSION_ACCEPTED,
    )

    # 5. NAV_DELAY with unsupported param5 (MAV_FRAME_MISSION, x=param5)
    # mask 0x0F; mission567=0x00 -> p5/p6/p7 not supported
    result = _upload_mission(mav, [
        _item(0, CMD_NAV_DELAY, MAV_FRAME_MISSION,
              p1=10.0, p2=NAN, p3=NAN, p4=NAN,
              x=42, y=0, z=0.0),
    ], timeout)
    _check(
        "NAV_DELAY unsupported param5 (x=42) -> INVALID_PARAM5",
        result, MAV_MISSION_INVALID_PARAM5,
    )

    # 6. NAV_DELAY valid (p1 set, p5/p6/p7 zero)
    result = _upload_mission(mav, [
        _item(0, CMD_NAV_DELAY, MAV_FRAME_MISSION,
              p1=10.0, p2=NAN, p3=NAN, p4=NAN,
              x=0, y=0, z=0.0),
    ], timeout)
    _check(
        "NAV_DELAY valid (p1=10, p5/p6/p7=0) -> ACCEPTED",
        result, MAV_MISSION_ACCEPTED,
    )

    # 7. NAV_TAKEOFF with unsupported param2 set (mask 0x08: only p4)
    result = _upload_mission(mav, [
        _item(0, CMD_NAV_TAKEOFF, MAV_FRAME_GLOBAL_INT,
              p1=NAN, p2=5.0, p3=NAN, p4=NAN,
              x=LAT, y=LON, z=ALT, current=1),
    ], timeout)
    _check(
        "NAV_TAKEOFF unsupported param2 -> INVALID_PARAM2",
        result, MAV_MISSION_INVALID_PARAM2,
    )


def run_command_tests(mav: Any, timeout: float) -> None:
    print("\n=== Command (COMMAND_LONG) tests ===")

    # 8. Valid COMPONENT_ARM_DISARM (p1=0 disarm, p2=0 no-force)
    result = _send_command(
        mav, CMD_COMPONENT_ARM_DISARM, timeout, p1=0.0, p2=0.0,
    )
    _check(
        "COMPONENT_ARM_DISARM valid params -> not DENIED",
        result != MAV_RESULT_DENIED, True,
    )

    # 9. COMPONENT_ARM_DISARM with unsupported param3 set (mask 0x03)
    result = _send_command(
        mav, CMD_COMPONENT_ARM_DISARM, timeout, p1=0.0, p2=0.0, p3=1.0,
    )
    _check(
        "COMPONENT_ARM_DISARM unsupported param3 -> DENIED",
        result, MAV_RESULT_DENIED,
    )

    # 10. NAV_RTL command with param1 set (mask 0x00)
    result = _send_command(mav, CMD_NAV_RTL, timeout, p1=1.0)
    _check(
        "NAV_RTL command unsupported param1 -> DENIED",
        result, MAV_RESULT_DENIED,
    )

    # 11. NAV_RTL command all params zero -> not DENIED
    result = _send_command(mav, CMD_NAV_RTL, timeout)
    _check(
        "NAV_RTL command all params zero -> not DENIED",
        result != MAV_RESULT_DENIED, True,
    )

    # 12. NAV_WAYPOINT command unsupported param3 (mask 0x0B)
    result = _send_command(
        mav, CMD_NAV_WAYPOINT, timeout, p1=0.0, p2=2.0, p3=1.0, p4=0.0,
    )
    _check(
        "NAV_WAYPOINT command unsupported param3 -> DENIED",
        result, MAV_RESULT_DENIED,
    )


# Entry point


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--url", default="udp://:14540",
        help="MAVLink connection URL (default: udp://:14540)",
    )
    parser.add_argument(
        "--timeout", type=float, default=5.0,
        help="Per-message receive timeout in seconds (default: 5)",
    )
    args = parser.parse_args()

    mav = connect(args.url)

    run_mission_tests(mav, args.timeout)
    run_command_tests(mav, args.timeout)

    passed = sum(_results)
    total = len(_results)
    print(f"\nResult: {passed}/{total} passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
