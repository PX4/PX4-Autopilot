"""AutopilotTester for the emergency and the contingency geofence.

Fence shapes use home-relative local (north, east) coordinates. Clearances are measured on
ground truth using the origin saved when the fence was uploaded.
"""

import math
import threading
from typing import Any, Callable, List, Optional, Tuple

from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import (
    MANUAL_CONTROL_RATE_HZ, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_NAV_WAYPOINT,
    MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT, log, raw_mission_item)
from autopilot_tester_loiter import AutopilotTesterLoiter, horizontal_distance
from geofence_geometry import (
    FenceKind, FenceShape, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, rectangle)
from geometry import CoordinateTransformation

MAV_CMD_NAV_VTOL_TAKEOFF = 84
MAV_MISSION_TYPE_FENCE = 1
MAV_RESULT_TEMPORARILY_REJECTED = 1

# Values of GF_ACTION and GF_CONT_ACTION.
ACTION_WARNING = 1
ACTION_HOLD = 2
ACTION_RETURN = 3
ACTION_LAND = 5

# How far past the edge the vehicle must be for a crossing to count.
CROSSING_DEPTH_M = 5.0
# How far inside the fence a recovered vehicle must be on the ground truth, beyond any error of the
# estimate it navigates with.
RECOVERED_DEPTH_M = 10.0

Local2 = Tuple[float, float]


class AutopilotTesterGeofence(AutopilotTesterLoiter):

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        super().__init__(speed_factor)
        self._fence_shapes: List[FenceShape] = []
        self._fence_origin: Optional[CoordinateTransformation] = None
        self._track_handle: Any = None
        self._track_lock = threading.Lock()
        self._min_fence_distance_m = math.inf
        self._max_home_distance_m = 0.0

    # ------------------------------------------------------------------
    # Fence, missions and commands
    # ------------------------------------------------------------------

    def upload_inclusion_rectangle(self, north_min_m: float, north_max_m: float,
                                   east_min_m: float, east_max_m: float) -> None:
        """Upload one inclusion rectangle as the geofence plan."""
        corners = rectangle(north_min_m, north_max_m, east_min_m, east_max_m)
        self.upload_fence([FenceShape(FenceKind.POLYGON_INCLUSION, vertices=corners)])

    def upload_fence(self, shapes: List[FenceShape]) -> None:
        """Upload the shapes as the geofence plan."""
        ct = self.get_coordinate_transformation()
        items: List[Any] = []
        commands = {
            FenceKind.POLYGON_INCLUSION: MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            FenceKind.POLYGON_EXCLUSION: MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            FenceKind.CIRCLE_INCLUSION: MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            FenceKind.CIRCLE_EXCLUSION: MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
        }
        for shape in shapes:
            polygon = shape.kind in (FenceKind.POLYGON_INCLUSION, FenceKind.POLYGON_EXCLUSION)
            for point in shape.vertices if polygon else [shape.center]:
                latitude_deg, longitude_deg = ct.global_from_local(*point)
                items.append(raw_mission_item(
                    len(items), commands[shape.kind], MAV_FRAME_GLOBAL_INT,
                    x=round(latitude_deg * 1e7), y=round(longitude_deg * 1e7),
                    param1=float(len(shape.vertices)) if polygon else shape.radius_m,
                    # MAVSDK requires one current item even for a fence.
                    current=0 if items else 1,
                    autocontinue=0, mission_type=MAV_MISSION_TYPE_FENCE))

        self._mission_raw.upload_geofence(items)
        self._fence_shapes = shapes
        self._fence_origin = ct
        # PX4 needs time to load the fence.
        self.sleep_for(1)

    def upload_vtol_mission(self, waypoints: List[Local2], rel_alt_m: float) -> None:
        """Upload a VTOL takeoff followed by the waypoints, without a landing."""
        ct = self.get_coordinate_transformation()
        items = []
        for seq, waypoint in enumerate(waypoints):
            latitude_deg, longitude_deg = ct.global_from_local(*waypoint)
            items.append(raw_mission_item(
                seq, MAV_CMD_NAV_VTOL_TAKEOFF if seq == 0 else MAV_CMD_NAV_WAYPOINT,
                MAV_FRAME_GLOBAL_RELATIVE_ALT,
                x=round(latitude_deg * 1e7), y=round(longitude_deg * 1e7), z=rel_alt_m,
                current=1 if seq == 0 else 0))

        self.upload_mission_raw(items)

    def check_reposition_rejected(self, target: Local2, rel_alt_m: float, event_name: str) -> None:
        """Wait for Navigator's rejection after Commander acknowledges the Go-to."""
        with self.observe_event(name=event_name) as rejected:
            self.command_reposition(target, rel_alt_m)
            assert self.poll_condition_with_timeout(rejected.is_set, 5), \
                "Navigator did not report {}".format(event_name)

    def check_arming_is_denied(self) -> None:
        """One arming attempt, without the retries of arm(), that must be denied."""
        self.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, param1=1.0,
                               expected_result=MAV_RESULT_TEMPORARILY_REJECTED)

    def fly_in_position_mode_until_failsafe(self, flight_mode: FlightMode, forward: float = 0.5,
                                            throttle: float = 0.5, timeout_s: float = 60) -> None:
        """Hold the sticks in Position mode until a failsafe takes over, and check its flight mode.

        Half forward stick flies a multicopter forward, north after the take-off at Home. A
        throttle of 0.5 keeps the altitude."""
        self.fly_in_position_mode_until(lambda: self._telemetry.flight_mode() != FlightMode.POSCTL,
                                        forward, throttle, timeout_s)
        self.wait_for_flight_mode(flight_mode, 5)

    def fly_in_position_mode_until(self, condition: Callable[[], bool], forward: float = 0.5,
                                   throttle: float = 0.5, timeout_s: float = 60) -> None:
        """Hold the sticks in Position mode until condition() is true, then center them."""
        self._send_manual_control(1, 0.0, 0.0, 0.5, 0.0)
        self._manual_control.start_position_control()
        self._send_manual_control(1, 0.0, 0.0, 0.5, 0.0)
        assert self._telemetry.flight_mode() == FlightMode.POSCTL, "Not in Position mode"

        log("Flying with forward {} and throttle {}".format(forward, throttle))
        start_us = self._autopilot_time_us()
        while not condition():
            assert (self._autopilot_time_us() - start_us) / 1e6 < timeout_s, \
                "Pilot flight condition not met within {} s".format(timeout_s)
            self._manual_control.set_manual_control_input(forward, 0.0, throttle, 0.0)
            self.sleep_for(1.0 / MANUAL_CONTROL_RATE_HZ)
        self._manual_control.set_manual_control_input(0.0, 0.0, 0.5, 0.0)

    def recover_with_reposition(self, target: Local2, relative_altitude_m: float) -> None:
        """Send a multicopter outside the fence back inside with a Go-to. Hold keeps the Go-to
        target, also once the vehicle is back inside."""
        assert self.current_fence_distance_m() < 0.0, "Recovery must start outside the fence"
        with self.keep_manual_control_alive():
            self.command_reposition(target, relative_altitude_m)
            self.wait_for_flight_mode(FlightMode.HOLD, 5)
            with self.monitor_hold_position(expected_center=target):
                self.wait_for_position_ned(lambda s: horizontal_distance(s, target) < 3.0, 45)
        self.check_inside_fence(RECOVERED_DEPTH_M)

    # ------------------------------------------------------------------
    # How far the vehicle went
    # ------------------------------------------------------------------

    def start_tracking(self) -> None:
        """Record the extremes of the ground truth position from now on. Needs store_home()."""
        assert self._track_handle is None, "Already tracking"
        ct = CoordinateTransformation(*self._home_or_fail())

        self.reset_tracking_extremes()

        def on_ground_truth(ground_truth: Any, _user_data: Any) -> None:
            if math.isnan(ground_truth.latitude_deg) or math.isnan(ground_truth.longitude_deg):
                return
            north_m, east_m = ct.local_from_global(ground_truth.latitude_deg,
                                                   ground_truth.longitude_deg)
            with self._track_lock:
                self._max_home_distance_m = max(self._max_home_distance_m,
                                                math.hypot(north_m, east_m))
                if self._fence_shapes:
                    self._min_fence_distance_m = min(
                        self._min_fence_distance_m,
                        self.fence_distance_m(ground_truth.latitude_deg,
                                              ground_truth.longitude_deg))

        self._track_handle = self._telemetry.subscribe_ground_truth(on_ground_truth)

    def fence_distance_m(self, latitude_deg: float, longitude_deg: float) -> float:
        """Signed clearance from the uploaded fence, using the same origin as its vertices."""
        assert self._fence_shapes and self._fence_origin is not None, "No fence uploaded"
        north_m, east_m = self._fence_origin.local_from_global(latitude_deg, longitude_deg)
        return min(shape.clearance_m(north_m, east_m) for shape in self._fence_shapes)

    def current_fence_distance_m(self) -> float:
        """Signed clearance of the ground truth position from the uploaded fence."""
        position = self._telemetry.ground_truth()
        return self.fence_distance_m(position.latitude_deg, position.longitude_deg)

    def wait_until_outside_fence(self, depth_m: float, timeout_s: float) -> None:
        """Wait until the vehicle is more than depth_m outside the fence."""
        assert self.poll_condition_with_timeout(
            lambda: self.current_fence_distance_m() < -depth_m, timeout_s), \
            "Not {} m outside the fence within {} s".format(depth_m, timeout_s)

    def wait_until_inside_fence(self, depth_m: float, timeout_s: float) -> None:
        """Wait until the vehicle is more than depth_m inside the fence."""
        assert self.poll_condition_with_timeout(
            lambda: self.current_fence_distance_m() > depth_m, timeout_s), \
            "Not {} m inside the fence within {} s".format(depth_m, timeout_s)

    def check_inside_fence(self, depth_m: float) -> None:
        """Check that the vehicle is more than depth_m inside the fence now."""
        distance_m = self.current_fence_distance_m()
        assert distance_m > depth_m, \
            "{:.1f} m from the fence, expected more than {} m inside".format(distance_m, depth_m)

    def reset_tracking_extremes(self) -> None:
        """Start a new observation window without replacing the ground-truth subscription."""
        with self._track_lock:
            self._min_fence_distance_m = math.inf
            self._max_home_distance_m = 0.0

    def min_fence_distance_m(self) -> float:
        """Closest the vehicle came to the fence from inside; negative once it was outside."""
        with self._track_lock:
            return self._min_fence_distance_m

    def home_distance_m(self) -> float:
        """Current horizontal distance from Home."""
        return self.ground_truth_horizontal_distance_to(*self._home_or_fail())

    def max_home_distance_m(self) -> float:
        with self._track_lock:
            return self._max_home_distance_m
