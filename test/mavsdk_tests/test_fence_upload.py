"""Checks of the fence polygons in the upload of a geofence plan."""

import math
from dataclasses import dataclass, replace
from typing import Any, List

import pytest
from mavsdk.plugins.mission_raw import MissionRawResult  # type: ignore

from autopilot_tester import MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_INT, raw_mission_item
from autopilot_tester_geofence import MAV_MISSION_TYPE_FENCE, AutopilotTesterGeofence
from geofence_geometry import (
    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)

# Fence items stored by PX4.
STORED_ITEMS = 64

# MAVSDK reports MAV_MISSION_INVALID and MAV_MISSION_INVALID_PARAM1 to 7 as the same result.
INVALID = MissionRawResult.INVALID_ARGUMENT


@dataclass(frozen=True)
class FenceItem:
    command: int
    param1: float
    latitude: int
    longitude: int


@pytest.mark.multicopter
def test_fence_upload_rejects_malformed_polygons(tester_geofence: AutopilotTesterGeofence) -> None:
    """A polygon arrives one vertex at a time. An upload with a malformed polygon, or with an item
    that is not a fence item, is rejected before it replaces the active plan."""
    tester = tester_geofence
    tester.wait_until_ready()
    ct = tester.get_coordinate_transformation()

    def item(command: int, param1: float, north_m: float, east_m: float) -> FenceItem:
        latitude_deg, longitude_deg = ct.global_from_local(north_m, east_m)
        return FenceItem(command, param1, round(latitude_deg * 1e7), round(longitude_deg * 1e7))

    def upload(items: List[FenceItem]) -> Any:
        return tester.upload_fence_items([
            raw_mission_item(seq, item.command, MAV_FRAME_GLOBAL_INT, x=item.latitude,
                             y=item.longitude, param1=item.param1,
                             # MAVSDK requires a current item.
                             current=1 if seq == 0 else 0, autocontinue=0,
                             mission_type=MAV_MISSION_TYPE_FENCE)
            for seq, item in enumerate(items)])

    def stored() -> List[FenceItem]:
        return [FenceItem(item.command, item.param1, item.x, item.y)
                for item in tester.download_fence_items()]

    def reject(label: str, items: List[FenceItem], result: Any) -> None:
        assert upload(items) == result, label
        assert stored() == triangle, label + ": the active plan changed"

    # An inclusion triangle around Home.
    triangle = [item(MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, 3.0,
                     3000.0 * math.cos(angle), 3000.0 * math.sin(angle))
                for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)]
    assert upload(triangle) == MissionRawResult.SUCCESS
    assert stored() == triangle

    # Each vertex has the command and the vertex count of the first one.
    circle = item(MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, 50.0, 500.0, -500.0)
    for changed in (replace(triangle[1], command=MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION),
                    replace(triangle[1], param1=4.0), circle):
        label = "vertex replaced by {}".format(changed)
        reject(label, [triangle[0], changed, triangle[2]], INVALID)
        # The next upload starts without the polygon that the rejected one left unfinished.
        assert upload(triangle) == MissionRawResult.SUCCESS, label + ", next upload"

    # The vertex count is checked before it is used, and still rounded to the nearest integer.
    for count in (4.0, math.nan, math.inf, -math.inf, -1.0, 2.0, STORED_ITEMS + 0.5, 65536.0):
        reject("vertex count {}".format(count),
               [replace(vertex, param1=count) for vertex in triangle], INVALID)
    for count in (2.5, 3.4):
        assert upload([replace(vertex, param1=count) for vertex in triangle]) \
            == MissionRawResult.SUCCESS
        assert stored() == triangle, "vertex count {}".format(count)

    reject("item that is not a fence item",
           [replace(triangle[0], command=MAV_CMD_NAV_WAYPOINT, param1=0.0)],
           MissionRawResult.UNSUPPORTED)
