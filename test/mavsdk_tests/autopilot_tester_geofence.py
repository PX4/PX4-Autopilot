"""AutopilotTester for the emergency and the contingency geofence.

The plan fence is a rectangle in home-relative local (north, east) coordinates. How far the
vehicle went is taken from the ground truth, so a breach shows even if the estimate hides it.
"""

from contextlib import contextmanager
import json
import math
import threading
from typing import Any, Iterator, List, Optional, Tuple

from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import (
    MANUAL_CONTROL_RATE_HZ, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_NAV_WAYPOINT,
    MAV_CMD_REQUEST_MESSAGE,
    MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT, MissionOptions, log, raw_mission_item)
from autopilot_tester_loiter import AutopilotTesterLoiter
from geometry import CoordinateTransformation

MAV_CMD_NAV_VTOL_TAKEOFF = 84
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
MAV_MISSION_TYPE_FENCE = 1
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT = 87

# Values of GF_ACTION and GF_CONT_ACTION.
ACTION_WARNING = 1
ACTION_HOLD = 2
ACTION_RETURN = 3
ACTION_LAND = 5

Local2 = Tuple[float, float]


class AutopilotTesterGeofence(AutopilotTesterLoiter):

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        super().__init__(speed_factor)
        # north_min, north_max, east_min, east_max
        self._fence: Optional[Tuple[float, float, float, float]] = None
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
        """Upload one inclusion polygon as the geofence plan."""
        ct = self.get_coordinate_transformation()
        corners: List[Local2] = [(north_min_m, east_min_m), (north_max_m, east_min_m),
                                 (north_max_m, east_max_m), (north_min_m, east_max_m)]
        items = []
        for seq, corner in enumerate(corners):
            latitude_deg, longitude_deg = ct.global_from_local(*corner)
            items.append(raw_mission_item(
                seq, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAV_FRAME_GLOBAL_INT,
                x=round(latitude_deg * 1e7), y=round(longitude_deg * 1e7),
                param1=float(len(corners)),
                # MAVSDK requires a current item, even though it makes little sense for a fence.
                current=1 if seq == 0 else 0,
                autocontinue=0, mission_type=MAV_MISSION_TYPE_FENCE))

        self._mission_raw.upload_geofence(items)
        self._fence = (north_min_m, north_max_m, east_min_m, east_max_m)
        self._fence_origin = ct
        # PX4 needs time to load the fence.
        self.sleep_for(1)

    def upload_mission_to(self, north_m: float, options: MissionOptions) -> None:
        """Upload a mission from above Home to a point north of it."""
        self._upload_mission([(0.0, 0.0), (north_m, 0.0)], options)

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

    def select_mission_mode(self) -> None:
        """Switch to Mission mode. A disarmed vehicle accepts that even with a rejected mission."""
        assert self._try_start_mission(), "Could not switch to Mission mode"

    def check_arming_is_denied(self) -> None:
        """One arming attempt, without the retries of arm(). Sent as a command, because
        MAVSDK's arm() first switches a vehicle in Mission mode to Hold."""
        self.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, param1=1.0,
                               expected_result=MAV_RESULT_TEMPORARILY_REJECTED)

    def fly_in_position_mode_until_hold(self, forward: float, throttle: float,
                                        timeout_s: float) -> None:
        """Hold the sticks in Position mode until a failsafe takes over with Hold.
        A throttle of 0.5 keeps the altitude."""
        self._send_manual_control(1, 0.0, 0.0, 0.5, 0.0)
        self._manual_control.start_position_control()
        self._send_manual_control(1, 0.0, 0.0, 0.5, 0.0)
        assert self._telemetry.flight_mode() == FlightMode.POSCTL, "Not in Position mode"

        log("Flying with forward {} and throttle {}".format(forward, throttle))
        start_us = self._autopilot_time_us()
        while self._telemetry.flight_mode() != FlightMode.HOLD:
            assert (self._autopilot_time_us() - start_us) / 1e6 < timeout_s, \
                "No Hold within {} s".format(timeout_s)
            self._manual_control.set_manual_control_input(forward, 0.0, throttle, 0.0)
            self.sleep_for(1.0 / MANUAL_CONTROL_RATE_HZ)

    def keep_sticks_centered_for(self, duration_s: float) -> None:
        """Keep the manual control link alive without asking for anything."""
        self._send_manual_control(duration_s, 0.0, 0.0, 0.5, 0.0)

    def check_flight_mode_for(self, flight_mode: FlightMode, duration_s: float) -> None:
        start_us = self._autopilot_time_us()
        while (self._autopilot_time_us() - start_us) / 1e6 < duration_s:
            current = self._telemetry.flight_mode()
            assert current == flight_mode, \
                "Flight mode {}, expected {}".format(current, flight_mode)
            self.sleep_for(0.2)

    @contextmanager
    def monitor_hold_position(self) -> Iterator[Tuple[float, float]]:
        """Require Hold and an unchanged loiter center throughout the block.

        Yield the first Hold target as (latitude, longitude). PX4 publishes the loiter center
        in POSITION_TARGET_GLOBAL_INT, so moving along a fence edge is detected too.
        """
        centers: List[Tuple[int, int]] = []
        target_count = 0

        def on_target(message: Any, _user_data: Any) -> None:
            nonlocal target_count
            fields = json.loads(message.fields_json)
            center = (int(fields["lat_int"]), int(fields["lon_int"]))
            if not centers:
                centers.append(center)
            target_count += 1
            self.check_in_callback(center == centers[0],
                                   "Hold center changed from {} to {}".format(centers[0], center))

        def on_mode(flight_mode: FlightMode, _user_data: Any) -> None:
            self.check_in_callback(flight_mode == FlightMode.HOLD,
                                   "Flight mode {}, expected Hold".format(flight_mode))

        mode_handle = self._telemetry.subscribe_flight_mode(on_mode)
        target_handle = self._mavlink_direct.subscribe_message(
            "POSITION_TARGET_GLOBAL_INT", on_target)
        try:
            on_mode(self._telemetry.flight_mode(), None)
            self.send_command_long(MAV_CMD_REQUEST_MESSAGE,
                                   param1=MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT)
            assert self.poll_condition_with_timeout(lambda: bool(centers), 5), \
                "No Hold position target received"
            yield centers[0][0] / 1e7, centers[0][1] / 1e7
        finally:
            self._mavlink_direct.unsubscribe_message(target_handle)
            self._telemetry.unsubscribe_flight_mode(mode_handle)
        self._assert_no_callback_failures()
        assert target_count > 1, "No further Hold position targets received"

    # ------------------------------------------------------------------
    # How far the vehicle went
    # ------------------------------------------------------------------

    def start_tracking(self) -> None:
        """Record the extremes of the ground truth position from now on. Needs store_home()."""
        assert self._track_handle is None, "Already tracking"
        ct = CoordinateTransformation(*self._home_or_fail())

        with self._track_lock:
            self._min_fence_distance_m = math.inf
            self._max_home_distance_m = 0.0

        def on_ground_truth(ground_truth: Any, _user_data: Any) -> None:
            if math.isnan(ground_truth.latitude_deg) or math.isnan(ground_truth.longitude_deg):
                return
            north_m, east_m = ct.local_from_global(ground_truth.latitude_deg,
                                                   ground_truth.longitude_deg)
            with self._track_lock:
                self._max_home_distance_m = max(self._max_home_distance_m,
                                                math.hypot(north_m, east_m))
                if self._fence is not None:
                    self._min_fence_distance_m = min(
                        self._min_fence_distance_m,
                        self.fence_distance_m(ground_truth.latitude_deg,
                                              ground_truth.longitude_deg))

        self._track_handle = self._telemetry.subscribe_ground_truth(on_ground_truth)

    def fence_distance_m(self, latitude_deg: float, longitude_deg: float) -> float:
        """Signed clearance from the uploaded fence, using the same origin as its vertices."""
        assert self._fence is not None and self._fence_origin is not None, "No fence uploaded"
        north_m, east_m = self._fence_origin.local_from_global(latitude_deg, longitude_deg)
        north_min_m, north_max_m, east_min_m, east_max_m = self._fence
        return min(north_m - north_min_m, north_max_m - north_m,
                   east_m - east_min_m, east_max_m - east_m)

    def min_fence_distance_m(self) -> float:
        """Closest the vehicle came to the fence from inside; negative once it was outside."""
        with self._track_lock:
            return self._min_fence_distance_m

    def home_distance_m(self) -> float:
        """Current horizontal distance from Home."""
        return self._ground_truth_horizontal_distance_to(*self._home_or_fail())

    def max_home_distance_m(self) -> float:
        with self._track_lock:
            return self._max_home_distance_m
