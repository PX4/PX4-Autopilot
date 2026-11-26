"""Fence level items in the upload and download of a geofence plan."""

import math
from dataclasses import dataclass, replace
from typing import Any, List

import pytest
from mavsdk.plugins.mission_raw import MissionRawResult  # type: ignore

from autopilot_tester import MAV_FRAME_GLOBAL_INT, MAV_FRAME_MISSION, raw_mission_item
from autopilot_tester_geofence import MAV_MISSION_TYPE_FENCE, AutopilotTesterGeofence
from geofence_geometry import (
    FENCE_LEVEL_CONTINGENCY, FENCE_LEVEL_EMERGENCY, MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
    MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, MAV_CMD_NAV_FENCE_LEVEL,
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)

MAV_CMD_NAV_FENCE_RETURN_POINT = 5000

# Fence items stored by a default build (CONFIG_NUM_FENCE_ITEMS_SUPPORTED).
STORED_ITEMS = 64

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
    """Level items are stored and returned with the areas. Other levels and a plan that does not
    fit the storage are rejected, and the active plan stays as it was. Which areas a level item
    applies to is unit tested in Navigator."""
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

    # Level items count toward the storage, and one polygon may fill the rest.
    full = area(polygon, FENCE_LEVEL_CONTINGENCY, STORED_ITEMS - 1)
    round_trip("full storage", full)
    return_point = replace(full[1], command=MAV_CMD_NAV_FENCE_RETURN_POINT, param1=0.0)
    reject("one item more than PX4 stores", full + [return_point], NO_SPACE, full)
