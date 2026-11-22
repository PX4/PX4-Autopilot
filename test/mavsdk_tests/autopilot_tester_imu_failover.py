"""AutopilotTester for estimator instance failover on SIH with two IMUs.

Needs SIH_IMU_COUNT 2, EKF2_MULTI_IMU 2 and SENS_IMU_MODE 0, so that each IMU feeds an estimator
instance of its own.

No MAVLink message carries the instance the estimator selector flies on, so it is read from
estimator_selector_status over the MAVLink shell. The IMU data is read from SCALED_IMU and
SCALED_IMU2, which carry vehicle_imu instance 0 and 1 and so show any injected IMU failure. The
vehicle itself is judged against the SIH ground truth, not against the estimate the faults
disturb.
"""

import json
import math
import re
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from mavsdk.plugins.mission import MissionError  # type: ignore
from mavsdk.plugins.shell import Device, Shell  # type: ignore

from autopilot_tester import AutopilotTester, log
from geometry import Vec3, point_to_segment_distance

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAVLINK_MSG_ID_SCALED_IMU = 26
MAVLINK_MSG_ID_SCALED_IMU2 = 116

# Host time: the shell answers at the pace of the host, not of the simulation.
SHELL_REPLY_TIMEOUT_S = 5.0
SHELL_POLL_INTERVAL_S = 0.02

# Acceleration of one SCALED_IMU or SCALED_IMU2 sample, x, y and z in mG.
AccelSample = Tuple[float, float, float]


class AutopilotTesterImuFailover(AutopilotTester):

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        super().__init__(speed_factor)
        self._shell: Any = None
        self._shell_lock = threading.Lock()
        self._shell_output = ""

        self._scaled_imu_rate_hz = 0.0

        self._altitude_lock = threading.Lock()
        self._altitude_handle: Any = None
        self._altitude_reference_m = math.nan
        self._altitude_latest_m = math.nan
        self._altitude_lowest_m = 0.0
        self._altitude_highest_m = 0.0
        self._altitude_samples = 0
        self._horizontal_farthest_m = 0.0

        self._corridor_lock = threading.Lock()
        self._corridor_handle: Any = None
        self._corridor_radius_m = 0.0
        self._corridor_worst_m = 0.0
        self._corridor_leg_samples: List[int] = []

    def connect(self, url: str) -> None:
        super().connect(url)
        self._shell = Shell(self._system)
        # Without a subscriber MAVSDK does not ask for a response, and PX4 closes the shell
        # before it has run the command.
        self._shell.subscribe_receive(self._on_shell_output)

    def autopilot_time_s(self) -> float:
        """Simulation time as the autopilot reports it, the clock sleep_for() waits on."""
        return self._autopilot_time_us() * 1e-6

    # ------------------------------------------------------------------
    # Estimator selector
    # ------------------------------------------------------------------

    def _on_shell_output(self, receive: Any, _user_data: Any) -> None:
        with self._shell_lock:
            self._shell_output += receive.data

    def estimator_selector_status(self, *fields: str) -> Dict[str, int]:
        """Integer fields of estimator_selector_status, all from one `listener` reply over the
        MAVLink shell. Empty if the shell did not answer in time."""
        # The reply arrives in chunks on a MAVSDK thread, so wait until every field and the end
        # of its line are there.
        patterns = {field: re.compile(r"\b{}:\s*(\d+)[ \t]*\r?\n".format(re.escape(field)))
                    for field in fields}

        with self._shell_lock:
            self._shell_output = ""

        self._shell.send("listener estimator_selector_status -n 1\n", Device.SHELL)

        deadline = time.monotonic() + SHELL_REPLY_TIMEOUT_S

        while time.monotonic() < deadline:
            with self._shell_lock:
                matches = {field: pattern.search(self._shell_output)
                           for field, pattern in patterns.items()}

            if all(matches.values()):
                return {field: int(match.group(1))
                        for field, match in matches.items() if match is not None}

            time.sleep(SHELL_POLL_INTERVAL_S)

        with self._shell_lock:
            log("No {} in the shell reply: {!r}".format(", ".join(fields), self._shell_output))

        return {}

    def primary_estimator_instance(self) -> int:
        """The estimator instance the selector flies on. -1 if the shell did not answer."""
        return self.estimator_selector_status("primary_instance").get("primary_instance", -1)

    # ------------------------------------------------------------------
    # IMU data
    # ------------------------------------------------------------------

    def request_scaled_imus(self, rate_hz: float) -> None:
        """Stream SCALED_IMU and SCALED_IMU2 at rate_hz. PX4 does not stream them on this link
        by default."""
        for message_id in (MAVLINK_MSG_ID_SCALED_IMU, MAVLINK_MSG_ID_SCALED_IMU2):
            self.send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, param1=float(message_id),
                                   param2=1e6 / rate_hz)

        self._scaled_imu_rate_hz = rate_hz

    def record_scaled_imus(self,
                           duration_s: float) -> Tuple[List[AccelSample], List[AccelSample]]:
        """Record the SCALED_IMU and SCALED_IMU2 accelerations for duration_s.

        A stream that stopped arriving would leave nothing to compare and pass every check on
        it, so both have to arrive at half the requested rate or better."""
        assert self._scaled_imu_rate_hz > 0.0, "Call request_scaled_imus() first"

        names = ("SCALED_IMU", "SCALED_IMU2")
        lock = threading.Lock()
        samples: Dict[str, List[AccelSample]] = {name: [] for name in names}

        def recorder(name: str) -> Callable[[Any, Any], None]:
            def on_message(message: Any, _user_data: Any) -> None:
                fields = json.loads(message.fields_json)
                with lock:
                    samples[name].append((float(fields["xacc"]), float(fields["yacc"]),
                                          float(fields["zacc"])))
            return on_message

        handles = [self._mavlink_direct.subscribe_message(name, recorder(name)) for name in names]

        try:
            self.sleep_for(duration_s)

        finally:
            for handle in handles:
                self._mavlink_direct.unsubscribe_message(handle)

        with lock:
            recorded = {name: list(samples[name]) for name in names}

        for name in names:
            assert len(recorded[name]) >= self._scaled_imu_rate_hz * duration_s / 2, \
                "Only {} {} samples in {} s".format(len(recorded[name]), name, duration_s)

        return recorded["SCALED_IMU"], recorded["SCALED_IMU2"]

    # ------------------------------------------------------------------
    # Ground truth
    # ------------------------------------------------------------------

    def start_tracking_ground_truth_altitude(self) -> None:
        """Track how far the ground truth moves from where it is now, until
        stop_tracking_ground_truth_altitude(). The altitude offset is tracked signed, so that
        climbing and descending can be bounded separately. The horizontal distance is kept as
        the farthest the vehicle got, see ground_truth_horizontal_drift_m()."""
        start = self._telemetry.ground_truth()
        reference_m = float(start.absolute_altitude_m)
        assert math.isfinite(reference_m), "No ground truth altitude"
        assert math.isfinite(start.latitude_deg) and math.isfinite(start.longitude_deg), \
            "No ground truth position"

        ct = self.get_coordinate_transformation()
        reference_north_m, reference_east_m = ct.local_from_global(start.latitude_deg,
                                                                   start.longitude_deg)

        with self._altitude_lock:
            self._altitude_reference_m = reference_m
            self._altitude_latest_m = reference_m
            self._altitude_lowest_m = 0.0
            self._altitude_highest_m = 0.0
            self._altitude_samples = 0
            self._horizontal_farthest_m = 0.0

        def on_ground_truth(ground_truth: Any, _user_data: Any) -> None:
            altitude_m = float(ground_truth.absolute_altitude_m)

            if not (math.isfinite(altitude_m) and math.isfinite(ground_truth.latitude_deg)
                    and math.isfinite(ground_truth.longitude_deg)):
                return

            offset_m = altitude_m - reference_m
            north_m, east_m = ct.local_from_global(ground_truth.latitude_deg,
                                                   ground_truth.longitude_deg)
            distance_m = math.hypot(north_m - reference_north_m, east_m - reference_east_m)

            with self._altitude_lock:
                self._altitude_latest_m = altitude_m
                self._altitude_lowest_m = min(self._altitude_lowest_m, offset_m)
                self._altitude_highest_m = max(self._altitude_highest_m, offset_m)
                self._altitude_samples += 1
                self._horizontal_farthest_m = max(self._horizontal_farthest_m, distance_m)

        self._altitude_handle = self._telemetry.subscribe_ground_truth(on_ground_truth)

    def reset_ground_truth_altitude_samples(self) -> None:
        with self._altitude_lock:
            self._altitude_samples = 0

    def ground_truth_altitude_samples(self) -> int:
        """Ground truth samples since tracking started or since the last reset."""
        with self._altitude_lock:
            return self._altitude_samples

    def ground_truth_horizontal_drift_m(self) -> float:
        """The farthest the ground truth got, horizontally, from where it was when the tracking
        started."""
        with self._altitude_lock:
            return self._horizontal_farthest_m

    def stop_tracking_ground_truth_altitude(self) -> Tuple[float, float, float]:
        """Stop tracking. Return the lowest and the highest offset from the reference altitude
        and the last one, in m."""
        if self._altitude_handle is not None:
            self._telemetry.unsubscribe_ground_truth(self._altitude_handle)
            self._altitude_handle = None

        with self._altitude_lock:
            return (self._altitude_lowest_m, self._altitude_highest_m,
                    self._altitude_latest_m - self._altitude_reference_m)

    def start_tracking_mission_ground_truth(self, corridor_radius_m: float) -> None:
        """Measure how far the vehicle strays from the mission legs, until
        stop_tracking_mission_ground_truth(). Measured against the ground truth, so that an
        estimator jump cannot move the measurement.

        Horizontal only. The altitude is bounded on its own, and mixing an AMSL ground truth
        altitude with a mission relative altitude would compare two datums."""
        items = self._mission.download_mission().mission_items
        assert len(items) >= 2, "Mission has {} items, need at least 2".format(len(items))

        ct = self.get_coordinate_transformation()
        waypoints = [ct.local_from_global(item.latitude_deg, item.longitude_deg)
                     for item in items]

        with self._corridor_lock:
            self._corridor_radius_m = corridor_radius_m
            self._corridor_worst_m = 0.0
            # One counter per item, indexed by the item being flown to, so that every leg can
            # be shown to have been measured rather than only that something was.
            self._corridor_leg_samples = [0] * len(items)

        def leg_distance(position: Vec3, item: int) -> float:
            start = (waypoints[item - 1][0], waypoints[item - 1][1], 0.0)
            end = (waypoints[item][0], waypoints[item][1], 0.0)
            return point_to_segment_distance(position, start, end)

        def on_ground_truth(ground_truth: Any, _user_data: Any) -> None:
            # The item being flown to is read here rather than delivered with the sample, so a
            # sample queued just before a waypoint switch can arrive with the next item
            # current. Measuring against the leg being flown and the one before it, as finite
            # segments, keeps such a sample on the leg it belongs to.
            current = int(self._mission.mission_progress().current)

            if not 0 < current < len(waypoints):
                return

            if not (math.isfinite(ground_truth.latitude_deg)
                    and math.isfinite(ground_truth.longitude_deg)):
                return

            north_m, east_m = ct.local_from_global(ground_truth.latitude_deg,
                                                   ground_truth.longitude_deg)
            position = (north_m, east_m, 0.0)
            distance_m = leg_distance(position, current)

            if current >= 2:
                distance_m = min(distance_m, leg_distance(position, current - 1))

            with self._corridor_lock:
                self._corridor_worst_m = max(self._corridor_worst_m, distance_m)
                self._corridor_leg_samples[current] += 1

        self._corridor_handle = self._telemetry.subscribe_ground_truth(on_ground_truth)

    def stop_tracking_mission_ground_truth(self) -> None:
        """Stop measuring and check that every leg was measured and stayed in the corridor."""
        if self._corridor_handle is not None:
            self._telemetry.unsubscribe_ground_truth(self._corridor_handle)
            self._corridor_handle = None

        with self._corridor_lock:
            radius_m = self._corridor_radius_m
            worst_m = self._corridor_worst_m
            leg_samples = list(self._corridor_leg_samples[1:])

        log("worst ground truth distance from the mission legs {:.2f} m, samples per leg {}"
            .format(worst_m, leg_samples))

        # A maximum of zero because a leg was never measured would read as a pass.
        assert leg_samples and all(samples > 0 for samples in leg_samples), \
            "Not every mission leg was measured, samples per leg {}".format(leg_samples)
        assert worst_m < radius_m, \
            "Ground truth {:.2f} m from the mission legs, expected < {} m".format(
                worst_m, radius_m)

    # ------------------------------------------------------------------
    # Missions
    # ------------------------------------------------------------------

    def start_mission_and_wait_for_item(self, item: int, timeout_s: float) -> None:
        """Start the mission and wait until mission item `item` is being flown to."""
        assert self.poll_condition_with_timeout(self._try_start_mission, 3), \
            "Could not start mission"
        assert self.poll_condition_with_timeout(
            lambda: self._mission.mission_progress().current >= item, timeout_s), \
            "Mission item {} not reached within {} s".format(item, timeout_s)

    def mission_is_running(self) -> bool:
        """True once the mission has been started and until it has finished. False when the
        vehicle cannot tell."""
        try:
            return not bool(self._mission.is_mission_finished())
        except MissionError:
            return False
