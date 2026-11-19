"""Fence levels, param 3 of the fence commands, in the upload and download of a geofence plan."""

import math
from dataclasses import dataclass, replace
from typing import Any, List

import pytest
from mavsdk.plugins.mission_raw import MissionRawResult  # type: ignore

from autopilot_tester import MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_INT, raw_mission_item
from autopilot_tester_geofence import (
    FENCE_LEVEL_EMERGENCY, FENCE_LEVEL_CONTINGENCY, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
    MAV_MISSION_TYPE_FENCE, AutopilotTesterGeofence)

MAV_CMD_NAV_FENCE_RETURN_POINT = 5000
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002
MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003
MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004

LEVELS = (FENCE_LEVEL_EMERGENCY, FENCE_LEVEL_CONTINGENCY)
POINTS_PER_LEVEL = 64

# MAVSDK reports MAV_MISSION_INVALID and MAV_MISSION_INVALID_PARAM1 to 7 as the same result.
INVALID = MissionRawResult.INVALID_ARGUMENT
NO_SPACE = MissionRawResult.TOO_MANY_MISSION_ITEMS


@dataclass(frozen=True)
class FenceItem:
    command: int
    param1: float
    level: float
    latitude: int
    longitude: int


@pytest.mark.multicopter
def test_fence_levels_upload_and_download(tester_geofence: AutopilotTesterGeofence) -> None:
    """The level of each fence area is stored and returned. An upload that breaks the rules for
    levels, polygons or the points per level is rejected, and the active plan stays as it was."""
    tester = tester_geofence
    tester.wait_until_ready()
    ct = tester.get_coordinate_transformation()

    def shape(command: int, level: float) -> List[FenceItem]:
        """One area with Home on its allowed side: a triangle, or a circle."""
        if command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            param1, points = 3.0, [(-2000.0, -2000.0), (3000.0, 0.0), (-2000.0, 2000.0)]
        elif command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
            param1, points = 3.0, [(500.0, 0.0), (600.0, 100.0), (500.0, 200.0)]
        elif command == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            param1, points = 20000.0, [(0.0, 0.0)]
        else:
            param1, points = 50.0, [(500.0, -500.0)]

        items = []
        for north_m, east_m in points:
            latitude_deg, longitude_deg = ct.global_from_local(north_m, east_m)
            items.append(FenceItem(command, param1, level, round(latitude_deg * 1e7),
                                   round(longitude_deg * 1e7)))
        return items

    def upload(items: List[FenceItem]) -> Any:
        return tester.upload_fence_items([
            raw_mission_item(seq, item.command, MAV_FRAME_GLOBAL_INT, x=item.latitude,
                             y=item.longitude, param1=item.param1, param3=item.level,
                             # MAVSDK requires a current item.
                             current=1 if seq == 0 else 0, autocontinue=0,
                             mission_type=MAV_MISSION_TYPE_FENCE)
            for seq, item in enumerate(items)])

    def stored() -> List[FenceItem]:
        return [FenceItem(item.command, item.param1, item.param3, item.x, item.y)
                for item in tester.download_fence_items()]

    def round_trip(label: str, items: List[FenceItem]) -> None:
        assert upload(items) == MissionRawResult.SUCCESS, label
        assert stored() == items, label

    def reject(label: str, items: List[FenceItem], result: Any, active: List[FenceItem]) -> None:
        assert upload(items) == result, label
        assert stored() == active, label + ": the active plan changed"

    polygons = (MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION)
    polygon, circle = polygons[0], MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION

    mixed: List[FenceItem] = []
    for command in polygons + (MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, circle):
        for level in LEVELS:
            round_trip("command {}, level {}".format(command, level), shape(command, level))
            mixed += shape(command, level)
    round_trip("polygons and circles of both levels", mixed)

    # A level that is not set (NaN) is the emergency geofence, as before fence levels existed.
    for command in (polygon, circle):
        assert upload(shape(command, math.nan)) == MissionRawResult.SUCCESS
        assert stored() == shape(command, FENCE_LEVEL_EMERGENCY), "level not set"
    round_trip("polygons and circles of both levels again", mixed)

    # Only the levels 0 and 1 exist.
    for command in (polygon, circle):
        for level in (math.inf, -math.inf, -1.0, 0.5, 1.5, 2.0, 256.0):
            reject("command {}, level {}".format(command, level), shape(command, level),
                   INVALID, mixed)

    # All vertices of a polygon have the same level.
    for command in polygons:
        for first, second in (LEVELS, LEVELS[::-1]):
            items = shape(command, first)
            items[1] = replace(items[1], level=second)
            reject("command {} changes level {} to {}".format(command, first, second), items,
                   INVALID, mixed)

    # A new upload starts without the polygon that the rejected one left unfinished.
    plain = shape(polygon, FENCE_LEVEL_EMERGENCY)
    round_trip("plan after a rejected polygon", plain)

    # The vertex count is checked before it is used, and still rounded to the nearest integer.
    for count in (4.0, math.nan, math.inf, -math.inf, -1.0, 2.0, 64.5, 65.0, 65536.0):
        reject("vertex count {}".format(count), [replace(item, param1=count) for item in plain],
               INVALID, plain)
    for count in (2.5, 3.4):
        assert upload([replace(item, param1=count) for item in plain]) == MissionRawResult.SUCCESS
        assert stored() == plain, "vertex count {}".format(count)

    for changed in (replace(plain[1], command=polygons[1]), replace(plain[1], param1=4.0),
                    shape(circle, FENCE_LEVEL_EMERGENCY)[0]):
        reject("polygon vertex replaced by {}".format(changed), [plain[0], changed, plain[2]],
               INVALID, plain)

    return_point = replace(plain[0], command=MAV_CMD_NAV_FENCE_RETURN_POINT, param1=0.0)
    reject("command that is not a fence item",
           [replace(return_point, command=MAV_CMD_NAV_WAYPOINT)], MissionRawResult.UNSUPPORTED,
           plain)

    # Each polygon vertex and each circle takes one of the 64 points of its level.
    for level in LEVELS:
        full = shape(circle, level) * POINTS_PER_LEVEL
        round_trip("64 points at level {}".format(level), full)
        reject("65 points at level {}".format(level), full + full[:1], NO_SPACE, full)
        reject("polygon beyond the points left at level {}".format(level),
               full[:-2] + shape(polygon, level), NO_SPACE, full)

    # A return point counts as an emergency geofence point.
    geofence = shape(circle, FENCE_LEVEL_EMERGENCY) * (POINTS_PER_LEVEL - 1) + [return_point]
    round_trip("63 points and a return point", geofence)
    reject("64 points and a return point", geofence[:1] + geofence, NO_SPACE, geofence)

    both = [shape(circle, LEVELS[i % 2])[0] for i in range(2 * POINTS_PER_LEVEL)]
    round_trip("64 points at each level", both)
    for level in LEVELS:
        reject("128 points, 65 of them at level {}".format(level),
               shape(circle, level) * (POINTS_PER_LEVEL + 1)
               + shape(circle, 1.0 - level) * (POINTS_PER_LEVEL - 1), NO_SPACE, both)
    reject("129 points", both + both[:1], NO_SPACE, both)
