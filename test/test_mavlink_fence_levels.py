#!/usr/bin/env python3
"""Exercise fence levels through the MAVLink upload/download protocol.

Run against disposable SITL: this test replaces and clears its fence plan.
Requires pymavlink and a MAVLink 2 connection, for example:

    python3 test/test_mavlink_fence_levels.py --url udpin:127.0.0.1:14550
"""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass, replace

# Fence mission_type is a MAVLink 2 extension.
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402


@dataclass(frozen=True)
class FenceItem:
    command: int
    param1: float
    level: float
    latitude: int
    longitude: int


class FenceProtocol:
    def __init__(self, connection, timeout):
        self.connection = connection
        self.timeout = timeout
        self.mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE

    def receive(self, types):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type=types, blocking=True,
                timeout=max(0.0, deadline - time.monotonic()),
            )
            if message is not None and message.mission_type == self.mission_type:
                return message
        raise TimeoutError(f"Timed out waiting for {types}")

    def upload(self, items):
        connection = self.connection
        connection.mav.mission_count_send(
            connection.target_system, connection.target_component,
            len(items), self.mission_type,
        )
        deadline = time.monotonic() + self.timeout * (len(items) + 2)
        while time.monotonic() < deadline:
            message = self.receive(["MISSION_REQUEST_INT", "MISSION_ACK"])
            if message.get_type() == "MISSION_ACK":
                return message.type
            if message.seq >= len(items):
                raise AssertionError(f"Requested unexpected item {message.seq}")
            item = items[message.seq]
            connection.mav.mission_item_int_send(
                connection.target_system, connection.target_component,
                message.seq, mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                item.command, 0, 0,
                item.param1, 0.0, item.level, 0.0,
                item.latitude, item.longitude, 0.0, self.mission_type,
            )
        raise TimeoutError("Fence upload did not finish")

    def download(self):
        connection = self.connection
        connection.mav.mission_request_list_send(
            connection.target_system, connection.target_component,
            self.mission_type,
        )
        count = self.receive(["MISSION_COUNT"]).count
        items = []
        for sequence in range(count):
            connection.mav.mission_request_int_send(
                connection.target_system, connection.target_component,
                sequence, self.mission_type,
            )
            message = self.receive(["MISSION_ITEM_INT"])
            if message.seq != sequence:
                raise AssertionError(f"Received item {message.seq}, expected {sequence}")
            items.append(FenceItem(message.command, message.param1, message.param3, message.x, message.y))
        connection.mav.mission_ack_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_MISSION_ACCEPTED, self.mission_type,
        )
        return items


def shape(command, level):
    if command in (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                   mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION):
        vertices = ((474000000, 85000000), (474010000, 85010000), (474000000, 85020000))
        if command == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            vertices = ((473000000, 85000000), (474500000, 85000000), (474000000, 86500000))
        return [FenceItem(command, 3.0, level, lat, lon) for lat, lon in vertices]
    if command == mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
        return [FenceItem(command, 20000.0, level, 473977420, 85456060)]
    return [FenceItem(command, 50.0, level, 474000000, 85000000)]


def run_tests(protocol):
    accepted = mavutil.mavlink.MAV_MISSION_ACCEPTED
    polygon_inclusion = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
    polygon_exclusion = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
    circle_inclusion = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
    circle_exclusion = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
    commands = (polygon_inclusion, polygon_exclusion, circle_inclusion, circle_exclusion)
    checks = 0

    def check(label, actual, expected):
        nonlocal checks
        if actual != expected:
            raise AssertionError(f"{label}: got {actual!r}, expected {expected!r}")
        checks += 1
        print(f"PASS {label}")

    def round_trip(label, items):
        check(f"{label}: upload", protocol.upload(items), accepted)
        check(f"{label}: round trip", protocol.download(), items)

    def reject(label, items, expected_result, active_plan):
        check(f"{label}: reject", protocol.upload(items), expected_result)
        check(f"{label}: active plan unchanged", protocol.download(), active_plan)

    mixed = []
    for command in commands:
        for level in (0.0, 1.0):
            items = shape(command, level)
            round_trip(f"command {command}, level {level}", items)
            mixed.extend(items)
    round_trip("mixed hard/pre polygons and circles", mixed)

    # All four commands share the level parser; exercise invalid values once.
    for level in (math.nan, math.inf, -math.inf, -1.0, 0.5, 1.5, 2.0, 256.0):
        reject(f"invalid level {level}", shape(polygon_inclusion, level),
               mavutil.mavlink.MAV_MISSION_INVALID_PARAM3, mixed)

    for command in (polygon_inclusion, polygon_exclusion):
        for first, second in ((0.0, 1.0), (1.0, 0.0)):
            items = shape(command, first)
            items[1] = replace(items[1], level=second)
            reject(f"command {command}, polygon level changes {first} to {second}", items,
                   mavutil.mavlink.MAV_MISSION_INVALID_PARAM3, mixed)

    # A rejected partial polygon must not leak its state into the next transfer.
    legacy = shape(polygon_inclusion, 0.0)
    round_trip("legacy upload after a rejected polygon", legacy)
    reject("polygon longer than the upload", [replace(item, param1=4.0) for item in legacy],
           mavutil.mavlink.MAV_MISSION_INVALID_PARAM1, legacy)

    for count in (math.nan, math.inf, -1.0, 2.0, 3.5, 65.0):
        reject(f"invalid polygon vertex count {count}", [replace(item, param1=count) for item in legacy],
               mavutil.mavlink.MAV_MISSION_INVALID_PARAM1, legacy)

    for changed, expected in (
        (replace(legacy[1], command=polygon_exclusion), mavutil.mavlink.MAV_MISSION_INVALID),
        (replace(legacy[1], param1=4.0), mavutil.mavlink.MAV_MISSION_INVALID_PARAM1),
        (shape(circle_exclusion, 0.0)[0], mavutil.mavlink.MAV_MISSION_INVALID),
    ):
        reject("inconsistent polygon vertex", [legacy[0], changed, legacy[2]], expected, legacy)

    # Each level has its own 64-item allowance; every polygon vertex consumes one item.
    for level in (0.0, 1.0):
        single_level = shape(circle_exclusion, level) * 64
        round_trip(f"64 items at level {level}", single_level)
        reject(f"65 items at level {level}", single_level + single_level[:1],
               mavutil.mavlink.MAV_MISSION_NO_SPACE, single_level)
        reject(f"polygon exceeds remaining level {level} allowance",
               single_level[:-2] + shape(polygon_inclusion, level),
               mavutil.mavlink.MAV_MISSION_NO_SPACE, single_level)

    maximum = [shape(circle_exclusion, float(index % 2))[0] for index in range(128)]
    round_trip("64 items at each level", maximum)
    for level in (0.0, 1.0):
        unbalanced = shape(circle_exclusion, level) * 65 + shape(circle_exclusion, 1.0 - level) * 63
        reject(f"128 items with 65 at level {level}", unbalanced,
               mavutil.mavlink.MAV_MISSION_NO_SPACE, maximum)
    reject("129 total fence items", maximum + maximum[:1], mavutil.mavlink.MAV_MISSION_NO_SPACE, maximum)
    round_trip("clear both levels", [])
    print(f"{checks} checks passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="udpin:127.0.0.1:14550")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    connection = mavutil.mavlink_connection(args.url)
    try:
        if connection.wait_heartbeat(timeout=15) is None:
            raise TimeoutError("No SITL heartbeat")
        run_tests(FenceProtocol(connection, args.timeout))
    finally:
        connection.close()


if __name__ == "__main__":
    main()
