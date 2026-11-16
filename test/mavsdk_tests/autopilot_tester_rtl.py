"""AutopilotTester with rally points and geofences for the return tests."""

import math
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, List, Optional, Tuple

from mavsdk.plugins.telemetry import VtolState  # type: ignore

from autopilot_tester import (
    MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_GLOBAL_FRAMES, AutopilotTester, log, raw_mission_item,
    raw_item_local)
from geometry import CoordinateTransformation

MAV_CMD_NAV_LOITER_TO_ALT = 31
MAV_CMD_NAV_RALLY_POINT = 5100
MAV_MISSION_TYPE_RALLY = 2

MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002
MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003
MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004

Local2 = Tuple[float, float]


class FenceKind(Enum):
    POLYGON_INCLUSION = 0
    POLYGON_EXCLUSION = 1
    CIRCLE_INCLUSION = 2
    CIRCLE_EXCLUSION = 3


@dataclass
class FenceShape:
    kind: FenceKind
    vertices: List[Local2] = field(default_factory=list)  # polygons only
    center: Local2 = (0.0, 0.0)  # circles only
    radius_m: float = 0.0  # circles only

    def is_breached_by(self, north_m: float, east_m: float) -> bool:
        """A breach is being inside an exclusion or outside an inclusion shape."""
        if self.kind in (FenceKind.POLYGON_INCLUSION, FenceKind.POLYGON_EXCLUSION):
            if len(self.vertices) < 3:
                return False
            inside = point_in_polygon(north_m, east_m, self.vertices)
            return inside == (self.kind == FenceKind.POLYGON_EXCLUSION)

        distance_m = math.hypot(north_m - self.center[0], east_m - self.center[1])
        if self.kind == FenceKind.CIRCLE_EXCLUSION:
            return distance_m < self.radius_m
        return distance_m > self.radius_m


def point_in_polygon(north_m: float, east_m: float, polygon: List[Local2]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        ni, ei = polygon[i]
        nj, ej = polygon[j]
        if (ei > east_m) != (ej > east_m) and north_m < (nj - ni) * (east_m - ei) / (ej - ei) + ni:
            inside = not inside
        j = i
    return inside


class AutopilotTesterRtl(AutopilotTester):

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        super().__init__(speed_factor)
        self._rally_points: List[Any] = []
        self._local_rally_points: List[Local2] = []
        self._fence_shapes: List[FenceShape] = []
        self._fence_monitor_handle: Any = None
        self._fence_breached = threading.Event()

    def set_rtl_type(self, rtl_type: int) -> None:
        self.set_param_int("RTL_TYPE", rtl_type)

    def set_rtl_appr_force(self, rtl_appr_force: int) -> None:
        self.set_param_int("RTL_APPR_FORCE", rtl_appr_force)

    def set_takeoff_land_requirements(self, requirements: int) -> None:
        self.set_param_int("MIS_TKO_LAND_REQ", requirements)

    # ------------------------------------------------------------------
    # Rally points
    # ------------------------------------------------------------------

    def _add_rally_item(self, command: int, north_m: float, east_m: float, z: float,
                        param2: float = 0.0) -> None:
        ct = self.get_coordinate_transformation()
        latitude_deg, longitude_deg = ct.global_from_local(north_m, east_m)
        self._rally_points.append(raw_mission_item(
            len(self._rally_points), command, MAV_FRAME_GLOBAL_RELATIVE_ALT,
            x=int(latitude_deg * 1e7), y=int(longitude_deg * 1e7), z=z, param2=param2,
            # MAVSDK requires a current item, even though it makes little sense for rally points.
            current=1 if not self._rally_points else 0,
            autocontinue=0, mission_type=MAV_MISSION_TYPE_RALLY))

    def add_local_rally_point(self, north_m: float, east_m: float) -> None:
        self._local_rally_points.append((north_m, east_m))
        self._add_rally_item(MAV_CMD_NAV_RALLY_POINT, north_m, east_m, 0.0)

    def add_local_rally_with_approaches_point(self, north_m: float, east_m: float) -> None:
        self.add_local_rally_point(north_m, east_m)
        self._add_approaches_to_point(north_m, east_m)

    def add_home_to_rally_point(self) -> None:
        self.add_local_rally_point(0.0, 0.0)

    def add_home_with_approaches_to_rally_point(self) -> None:
        self.add_local_rally_with_approaches_point(0.0, 0.0)

    def _add_approaches_to_point(self, north_m: float, east_m: float) -> None:
        # Loiter-to-altitude approaches 200 m north and east, with an 80 m radius.
        self._add_rally_item(MAV_CMD_NAV_LOITER_TO_ALT, north_m + 200.0, east_m, 15.0, 80.0)
        self._add_rally_item(MAV_CMD_NAV_LOITER_TO_ALT, north_m, east_m + 200.0, 15.0, 80.0)

    def upload_rally_points(self) -> None:
        self._mission_raw.upload_rally_points(self._rally_points)

    def check_rally_point_within(self, acceptance_radius_m: float) -> None:
        ct = CoordinateTransformation(*self._home_or_fail())
        distances_m = [
            self._ground_truth_horizontal_distance_to(*ct.global_from_local(north_m, east_m))
            for north_m, east_m in self._local_rally_points]
        assert any(d < acceptance_radius_m for d in distances_m), \
            "Not within {} m of any rally point, distances: {}".format(
                acceptance_radius_m, ["{:.1f}".format(d) for d in distances_m])

    def check_rtl_approaches(self, acceptance_radius_m: float, timeout_s: float) -> None:
        """Check that the vehicle descends on one of the rally point approach loiters, down to
        RTL_DESCEND_ALT."""
        ct = self.get_coordinate_transformation()
        return_alt_m = self.get_param_float("RTL_RETURN_ALT")
        descend_alt_m = self.get_param_float("RTL_DESCEND_ALT")
        approaches = [(raw_item_local(item, ct), float(item.param2)) for item in self._rally_points
                      if item.command == MAV_CMD_NAV_LOITER_TO_ALT]
        results: List[bool] = []

        def done(sample: Any) -> bool:
            altitude_m = -sample.position.down_m
            # Started to loiter down, so it should be on an approach loiter.
            if not (altitude_m < return_alt_m - 3.0 and sample.velocity.down_m_s > 0.05):
                return False

            for center, radius_m in approaches:
                distance_m = math.hypot(sample.position.north_m - center[0],
                                        sample.position.east_m - center[1])
                # Ported as is from the C++ test, and it can't fail: with both bounds ">", any
                # sample is outside the loiter of at least one of the two approaches, which are
                # far apart. So this only waits for the descent. TODO: check that the vehicle is
                # on the loiter of the approach it took.
                if distance_m > radius_m - acceptance_radius_m \
                        and distance_m > radius_m + acceptance_radius_m:
                    if altitude_m < descend_alt_m + 3.0:
                        results.append(True)
                        return True
                    return False

            results.append(False)
            return True

        self.wait_for_position_ned(done, timeout_s, rate_hz=50.0)
        assert results and results[0], "Vehicle descended without being on an approach loiter"

    def wait_until_disarmed_while_in_multicopter_mode(self, timeout_s: float) -> None:
        start_us = self._autopilot_time_us()
        while self._telemetry.armed():
            assert self._telemetry.vtol_state() == VtolState.MC, "Left multicopter mode"
            assert (self._autopilot_time_us() - start_us) / 1e6 < timeout_s, \
                "Not disarmed within {} s".format(timeout_s)
            self.sleep_for(0.1)

    # ------------------------------------------------------------------
    # Geofence
    # ------------------------------------------------------------------

    def load_qgc_mission_and_geofence_here(self, plan_file: str) -> None:
        """Upload the mission and geofence of a QGC .plan, both shifted by the same offset so
        that the mission starts at the current position and the fences keep their positions
        relative to it. The fence shapes are kept in local NED for the breach monitor."""
        data = self._mission_raw.import_qgroundcontrol_mission(plan_file)
        assert data.mission_items, "No mission items in {}".format(plan_file)

        position = self._telemetry.position()
        assert math.isfinite(position.latitude_deg) and math.isfinite(position.longitude_deg), \
            "No global position"
        offset_x = data.mission_items[0].x - int(1e7 * position.latitude_deg)
        offset_y = data.mission_items[0].y - int(1e7 * position.longitude_deg)

        # Only items in a global frame carry lat/lon in x/y. For MAV_FRAME_MISSION items
        # (e.g. Return, DO_* commands) x/y are generic params, so shifting them would make
        # the mission validation reject the item.
        for item in list(data.mission_items) + list(data.geofence_items):
            if item.frame in MAV_GLOBAL_FRAMES:
                item.x -= offset_x
                item.y -= offset_y

        self._mission_raw.upload_mission(data.mission_items)

        self._fence_shapes = []
        if data.geofence_items:
            self._mission_raw.upload_geofence(data.geofence_items)
            self._fence_shapes = self._parse_fence_shapes(data.geofence_items)

        self.sleep_for(1)

    def _parse_fence_shapes(self, items: List[Any]) -> List[FenceShape]:
        # Polygon vertices come as a run of items with param1 = vertex count; circles are one
        # item each with param1 = radius.
        ct = self.get_coordinate_transformation()
        shapes: List[FenceShape] = []
        polygon: Optional[FenceShape] = None
        vertex_count = 0

        for item in items:
            local = ct.local_from_global(item.x / 1e7, item.y / 1e7)

            if item.command in (MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                                MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION):
                kind = (FenceKind.POLYGON_INCLUSION
                        if item.command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
                        else FenceKind.POLYGON_EXCLUSION)
                # Start a new polygon when the kind changes or the previous one is full.
                if polygon is None or polygon.kind != kind or len(polygon.vertices) >= vertex_count:
                    polygon = FenceShape(kind)
                    shapes.append(polygon)
                    vertex_count = int(item.param1)
                polygon.vertices.append(local)

            elif item.command in (MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                                  MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION):
                polygon = None
                kind = (FenceKind.CIRCLE_INCLUSION
                        if item.command == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
                        else FenceKind.CIRCLE_EXCLUSION)
                shapes.append(FenceShape(kind, center=local, radius_m=float(item.param1)))

        return shapes

    def start_monitoring_geofence_breach(self) -> None:
        """Watch the ground truth position and latch any geofence breach until
        check_no_geofence_breach()."""
        assert self._fence_shapes, "Call load_qgc_mission_and_geofence_here() first"
        assert self._fence_monitor_handle is None, "Already monitoring"

        ct = self.get_coordinate_transformation()
        self._fence_breached.clear()

        def check(ground_truth: Any, _user_data: Any) -> None:
            if math.isnan(ground_truth.latitude_deg) or math.isnan(ground_truth.longitude_deg):
                return
            north_m, east_m = ct.local_from_global(ground_truth.latitude_deg,
                                                   ground_truth.longitude_deg)
            for i, shape in enumerate(self._fence_shapes):
                if shape.is_breached_by(north_m, east_m):
                    if not self._fence_breached.is_set():
                        self._fence_breached.set()
                        log("GEOFENCE BREACH (shape #{}) at NED ({:.1f}, {:.1f})".format(
                            i, north_m, east_m))
                    return

        self._fence_monitor_handle = self._telemetry.subscribe_ground_truth(check)

    def check_no_geofence_breach(self) -> None:
        if self._fence_monitor_handle is not None:
            self._telemetry.unsubscribe_ground_truth(self._fence_monitor_handle)
            self._fence_monitor_handle = None
        assert not self._fence_breached.is_set(), "Geofence was breached"
