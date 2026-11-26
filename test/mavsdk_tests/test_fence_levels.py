"""Fence level items in the upload and download of a geofence plan."""

import math
from dataclasses import dataclass, replace
from typing import Any, List

import pytest
from mavsdk.plugins.mission_raw import MissionRawResult  # type: ignore

from autopilot_tester import (
    MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_INT, MAV_FRAME_MISSION, raw_mission_item)
from autopilot_tester_geofence import MAV_MISSION_TYPE_FENCE, AutopilotTesterGeofence
from geofence_geometry import (
    FENCE_LEVEL_CONTINGENCY, FENCE_LEVEL_EMERGENCY, MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
    MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, MAV_CMD_NAV_FENCE_LEVEL,
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)

MAV_CMD_NAV_FENCE_RETURN_POINT = 5000

# Default build: points of each fence level, and fence items stored.
POINTS_PER_LEVEL = 64
STORED_ITEMS = 2 * POINTS_PER_LEVEL

# MAVSDK reports MAV_MISSION_INVALID and MAV_MISSION_INVALID_PARAM1 to 7 as the same result.
INVALID = MissionRawResult.INVALID_ARGUMENT
NO_SPACE = MissionRawResult.TOO_MANY_MISSION_ITEMS


@dataclass(frozen=True)
class FenceItem:
    command: int
    param1: float
    latitude: int = 0
    longitude: int = 0
    frame: int = MAV_FRAME_GLOBAL_INT


def level_item(level: float) -> FenceItem:
    return FenceItem(MAV_CMD_NAV_FENCE_LEVEL, level, frame=MAV_FRAME_MISSION)


@pytest.mark.multicopter
def test_fence_levels_upload_and_download(tester_geofence: AutopilotTesterGeofence) -> None:
    """Level items are stored and returned with the areas. An upload that breaks the rules for
    level items, polygons or the points per level is rejected, and the active plan stays as it
    was. Which areas a level item applies to is unit tested in Navigator."""
    tester = tester_geofence
    tester.wait_until_ready()
    ct = tester.get_coordinate_transformation()

    def area(command: int, level: float = FENCE_LEVEL_EMERGENCY,
             vertices: int = 3) -> List[FenceItem]:
        """One area with Home on its allowed side, behind a level item if it is in the
        contingency geofence: a regular polygon, or a circle."""
        if command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            param1, center, radius_m = float(vertices), (0.0, 0.0), 3000.0
        elif command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
            param1, center, radius_m = float(vertices), (500.0, 0.0), 100.0
        elif command == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            param1, center, radius_m, vertices = 20000.0, (0.0, 0.0), 0.0, 1
        else:
            param1, center, radius_m, vertices = 50.0, (500.0, -500.0), 0.0, 1

        items = [level_item(level)] if level != FENCE_LEVEL_EMERGENCY else []
        for i in range(vertices):
            angle = 2.0 * math.pi * i / vertices
            latitude_deg, longitude_deg = ct.global_from_local(
                center[0] + radius_m * math.cos(angle), center[1] + radius_m * math.sin(angle))
            items.append(FenceItem(command, param1, round(latitude_deg * 1e7),
                                   round(longitude_deg * 1e7)))
        return items

    def upload(items: List[FenceItem]) -> Any:
        return tester.upload_fence_items([
            raw_mission_item(seq, item.command, item.frame, x=item.latitude, y=item.longitude,
                             param1=item.param1,
                             # MAVSDK requires a current item.
                             current=1 if seq == 0 else 0, autocontinue=0,
                             mission_type=MAV_MISSION_TYPE_FENCE)
            for seq, item in enumerate(items)])

    def stored() -> List[FenceItem]:
        return [FenceItem(item.command, item.param1, item.x, item.y, item.frame)
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
        for level in (FENCE_LEVEL_EMERGENCY, FENCE_LEVEL_CONTINGENCY):
            round_trip("command {}, level {}".format(command, level), area(command, level))
            mixed += area(command, level)
    # An area without a level item is in the emergency geofence, but a level item may say so.
    mixed += [level_item(FENCE_LEVEL_EMERGENCY)] + area(circle)
    round_trip("areas of both levels", mixed)

    # Only the levels 0 and 1 exist.
    for level in (math.nan, math.inf, -math.inf, -1.0, 0.5, 1.5, 2.0, 256.0):
        reject("level {}".format(level), [level_item(level)] + area(circle), INVALID, mixed)

    # A level item goes in front of an area, not between the vertices of a polygon.
    plain = area(polygon)
    reject("level item inside a polygon",
           plain[:1] + [level_item(FENCE_LEVEL_CONTINGENCY)] + plain[1:], INVALID, mixed)

    # A new upload starts without the polygon that the rejected one left unfinished.
    round_trip("plan after a rejected polygon", plain)

    # The vertex count is checked before it is used, and still rounded to the nearest integer.
    for count in (4.0, math.nan, math.inf, -math.inf, -1.0, 2.0, 64.5, 65.0, 65536.0):
        reject("vertex count {}".format(count), [replace(item, param1=count) for item in plain],
               INVALID, plain)
    for count in (2.5, 3.4):
        assert upload([replace(item, param1=count) for item in plain]) == MissionRawResult.SUCCESS
        assert stored() == plain, "vertex count {}".format(count)

    for changed in (replace(plain[1], command=polygons[1]), replace(plain[1], param1=4.0),
                    area(circle)[0]):
        reject("polygon vertex replaced by {}".format(changed), [plain[0], changed, plain[2]],
               INVALID, plain)

    return_point = replace(plain[0], command=MAV_CMD_NAV_FENCE_RETURN_POINT, param1=0.0)
    reject("command that is not a fence item",
           [replace(return_point, command=MAV_CMD_NAV_WAYPOINT)], MissionRawResult.UNSUPPORTED,
           plain)

    # Each polygon vertex and each circle takes one of the 64 points of its level.
    emergency = area(circle) * POINTS_PER_LEVEL
    round_trip("64 emergency geofence points", emergency)
    reject("65 emergency geofence points", emergency + area(circle), NO_SPACE, emergency)
    reject("polygon beyond the emergency geofence points left", emergency[:-2] + area(polygon),
           NO_SPACE, emergency)

    contingency = area(polygon, FENCE_LEVEL_CONTINGENCY, POINTS_PER_LEVEL)
    round_trip("64 contingency geofence points", contingency)
    reject("65 contingency geofence points",
           contingency + area(circle, FENCE_LEVEL_CONTINGENCY), NO_SPACE, contingency)

    # Level items and the return point take no points of a level, only storage.
    round_trip("64 emergency geofence points and a return point", emergency + [return_point])
    # A level item sets the next area only, so the points after it are in the emergency geofence.
    full = area(polygon, FENCE_LEVEL_CONTINGENCY, POINTS_PER_LEVEL - 1) + emergency
    assert len(full) == STORED_ITEMS
    round_trip("full storage", full)
    reject("one item more than PX4 stores", full + [return_point], NO_SPACE, full)
