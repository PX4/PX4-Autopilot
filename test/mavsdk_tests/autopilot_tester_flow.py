"""AutopilotTester for optical flow aided navigation (sihsim_quadx_flow, sihsim_standard_vtol_flow).

The flow models fly without GNSS fusion, so the estimator never picks a global origin on its own
and everything that needs a global position (arming checks, home position, orbit center) stays
unavailable until a GCS provides one.

Flow quality can only be judged indirectly: the velocity the estimator reports is compared against
the SIH ground truth velocity, which is streamed out as HIL_STATE_QUATERNION.
"""

import json
import math
import threading
import time
from typing import Any, List, Optional, Tuple

from autopilot_tester import AutopilotTester, log
from geometry import CoordinateTransformation

AXES = ("north", "east", "down")

MAV_CMD_DO_ORBIT = 34

# Wall time between SET_GPS_GLOBAL_ORIGIN resends while the estimator has not taken the origin.
ORIGIN_RESEND_INTERVAL_S = 1.0

# ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER: the nose follows the circle center, so the
# heading turns by a full revolution per orbit.
ORBIT_YAW_FACE_CENTER = 0.0

# Cap on the recorded ground truth trajectory so that a hanging test cannot grow it without bound.
MAX_GROUND_TRUTH_SAMPLES = 100000

Local2 = Tuple[float, float]


def _heading_from_quaternion(q: List[float]) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class AutopilotTesterFlow(AutopilotTester):

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        super().__init__(speed_factor)
        self._velocity_lock = threading.Lock()
        self._estimate_valid = False
        self._estimated_velocity_m_s = [0.0, 0.0, 0.0]
        self._error_sum_m_s = [0.0, 0.0, 0.0]
        self._error_max_m_s = [0.0, 0.0, 0.0]
        self._error_samples = 0
        self._ground_truth_lla: List[Tuple[float, float]] = []
        self._heading_valid = False
        self._previous_heading_rad = 0.0
        self._heading_swept_rad = 0.0
        self._comparison_active = False
        self._position_velocity_handle: Any = None
        self._ground_truth_handle: Any = None

    # ------------------------------------------------------------------
    # EKF origin / arming
    # ------------------------------------------------------------------

    def _home_frame(self) -> CoordinateTransformation:
        return CoordinateTransformation(*self._home_or_fail())

    def set_ekf_origin_to_home(self, timeout_s: float = 20) -> None:
        """Set the EKF origin to the stored (ground truth) home position with
        SET_GPS_GLOBAL_ORIGIN and wait until the vehicle reports a global position. Requires
        store_home() to have run."""
        home_latitude_deg, home_longitude_deg = self._home_or_fail()
        origin = {
            "target_system": self._target_system_id(),
            "latitude": round(home_latitude_deg * 1e7),
            "longitude": round(home_longitude_deg * 1e7),
            "altitude": round(self.home_absolute_altitude_m() * 1e3),
            "time_usec": 0,
        }

        # The SITL startup script starts the estimator after MAVLink, and a subscriber that joins
        # late only sees the newest vehicle command. SET_GPS_GLOBAL_ORIGIN is not acknowledged, so
        # it is resent until it is taken. The estimator only publishes a global position once it
        # has an origin, so a finite global position confirms it.
        log("Waiting for the EKF origin to be set")
        last_sent_s = -math.inf

        def got_origin() -> bool:
            nonlocal last_sent_s
            position = self._telemetry.position()
            if math.isfinite(position.latitude_deg) and math.isfinite(position.longitude_deg):
                return True

            now_s = time.monotonic()
            if now_s - last_sent_s >= ORIGIN_RESEND_INTERVAL_S:
                self.send_mavlink_message("SET_GPS_GLOBAL_ORIGIN", origin)
                last_sent_s = now_s
            return False

        assert self.poll_condition_with_timeout(got_origin, timeout_s), \
            "EKF origin not set within {} s".format(timeout_s)

    def wait_until_armable(self, timeout_s: float = 60) -> None:
        """Wait until the autopilot itself reports that it can be armed.

        wait_until_ready() cannot be used on a flow only vehicle: MAVSDK derives both the global
        and the local position health from the GPS / optical flow / vision flags in SYS_STATUS,
        and PX4 only ever fills in the GPS one, so health_all_ok() stays false no matter how good
        the flow aided estimate is."""
        log("Waiting for the vehicle to be armable")
        assert self.poll_condition_with_timeout(
            lambda: bool(self._telemetry.health().is_armable), timeout_s), \
            "Not armable within {} s".format(timeout_s)

    # ------------------------------------------------------------------
    # Orbit
    # ------------------------------------------------------------------

    def command_orbit(self, center: Local2, radius_m: float, velocity_m_s: float,
                      rel_alt_m: float) -> None:
        """Command an orbit (MAV_CMD_DO_ORBIT) around a home-relative center. On a multicopter it
        is flown with the nose pointing at the center so that the heading turns with the vehicle,
        on a fixed wing it turns into a loiter of that radius. Pass NaN as the velocity to leave
        the tangential velocity to the vehicle."""
        latitude_deg, longitude_deg = self._home_frame().global_from_local(*center)
        self.send_command_int(
            MAV_CMD_DO_ORBIT, param1=radius_m, param2=velocity_m_s,
            param3=ORBIT_YAW_FACE_CENTER, param4=float("nan"),
            x=round(latitude_deg * 1e7), y=round(longitude_deg * 1e7),
            z=self.home_absolute_altitude_m() + rel_alt_m)

    def tangent_circle_center(self, radius_m: float) -> Local2:
        """Center of a clockwise circle of radius_m that passes through the current position
        with the current velocity already tangent to it (center() is 90 degrees clockwise of
        the velocity, matching command_orbit's positive-radius/clockwise convention).

        Commanding an orbit around this center is joined immediately instead of being flown out
        to from wherever it happened to be requested."""
        north_m, east_m, _down_m = self.get_current_position_ned()
        velocity = self._telemetry.position_velocity_ned().velocity
        speed_m_s = math.hypot(velocity.north_m_s, velocity.east_m_s)
        assert speed_m_s > 1.0, \
            "Not moving fast enough to determine a heading: {:.2f} m/s".format(speed_m_s)
        north_unit = velocity.north_m_s / speed_m_s
        east_unit = velocity.east_m_s / speed_m_s
        return (north_m - radius_m * east_unit, east_m + radius_m * north_unit)

    def wait_until_on_circle(self, center: Local2, radius_m: float, tolerance_m: float,
                             timeout_s: float) -> None:
        """Wait until the vehicle is on the circle around center, i.e. its distance to the
        center is within tolerance_m of radius_m.

        Flying out to the circle only crosses the band once and then stays on it, so the first
        time the vehicle is in the band it is established on it."""
        log("Waiting for the vehicle to get onto the circle")

        def on_circle() -> bool:
            position = self._telemetry.position_velocity_ned().position
            distance_m = math.hypot(position.north_m - center[0], position.east_m - center[1])
            return bool(abs(distance_m - radius_m) < tolerance_m)

        assert self.poll_condition_with_timeout(on_circle, timeout_s), \
            "Did not reach the circle within {} s".format(timeout_s)

    # ------------------------------------------------------------------
    # Ground truth comparison
    # ------------------------------------------------------------------

    def start_ground_truth_comparison(self, rate_hz: float = 30.0) -> None:
        """Start comparing the estimated NED velocity against the ground truth velocity, and
        start tracking how far the (ground truth) heading turns. Both streams are requested at
        rate_hz so that the two samples being compared are close in time."""
        self.stop_ground_truth_comparison()

        self._telemetry.set_rate_position_velocity_ned(rate_hz)
        self._telemetry.set_rate_ground_truth(rate_hz)

        with self._velocity_lock:
            self._estimate_valid = False
            self._error_sum_m_s = [0.0, 0.0, 0.0]
            self._error_max_m_s = [0.0, 0.0, 0.0]
            self._error_samples = 0
            self._heading_valid = False
            self._heading_swept_rad = 0.0
            self._ground_truth_lla = []

        def on_position_velocity_ned(sample: Any, _user_data: Any) -> None:
            with self._velocity_lock:
                self._estimated_velocity_m_s = [
                    sample.velocity.north_m_s, sample.velocity.east_m_s,
                    sample.velocity.down_m_s]
                self._estimate_valid = True

        def on_ground_truth(message: Any, _user_data: Any) -> None:
            fields = json.loads(message.fields_json)
            # HIL_STATE_QUATERNION carries the ground truth velocity in cm/s.
            truth_m_s = (fields["vx"] * 1e-2, fields["vy"] * 1e-2, fields["vz"] * 1e-2)
            heading_rad = _heading_from_quaternion(fields["attitude_quaternion"])

            with self._velocity_lock:
                if len(self._ground_truth_lla) < MAX_GROUND_TRUTH_SAMPLES:
                    self._ground_truth_lla.append(
                        (fields["lat"] * 1e-7, fields["lon"] * 1e-7))

                if self._heading_valid:
                    # Signed, so that yaw oscillations cancel out instead of adding up to a turn.
                    self._heading_swept_rad += math.remainder(
                        heading_rad - self._previous_heading_rad, 2.0 * math.pi)

                self._previous_heading_rad = heading_rad
                self._heading_valid = True

                if not self._estimate_valid:
                    return

                for i in range(3):
                    error = abs(self._estimated_velocity_m_s[i] - truth_m_s[i])
                    self._error_sum_m_s[i] += error
                    self._error_max_m_s[i] = max(self._error_max_m_s[i], error)

                self._error_samples += 1

        self._position_velocity_handle = self._telemetry.subscribe_position_velocity_ned(
            on_position_velocity_ned)
        self._ground_truth_handle = self._mavlink_direct.subscribe_message(
            "HIL_STATE_QUATERNION", on_ground_truth)
        self._comparison_active = True

    def stop_ground_truth_comparison(self) -> None:
        """Stop the comparison. Safe to call when it is not running."""
        if not self._comparison_active:
            return

        self._comparison_active = False
        self._telemetry.unsubscribe_position_velocity_ned(self._position_velocity_handle)
        self._mavlink_direct.unsubscribe_message(self._ground_truth_handle)

    def _heading_swept_deg(self) -> float:
        with self._velocity_lock:
            return math.degrees(abs(self._heading_swept_rad))

    def wait_until_heading_swept(self, min_sweep_deg: float, timeout_s: float) -> None:
        """Wait until the heading has turned by min_sweep_deg in total since the comparison
        started."""
        log("Waiting for the heading to turn by {} deg".format(min_sweep_deg))
        assert self.poll_condition_with_timeout(
            lambda: self._heading_swept_deg() > min_sweep_deg, timeout_s), \
            "Heading did not turn by {} deg within {} s".format(min_sweep_deg, timeout_s)

    def check_ground_truth_on_circle(self, center: Local2, radius_m: float,
                                     tolerance_m: float) -> None:
        """Assert the ground truth trajectory stayed on the commanded circle: every recorded
        sample was within tolerance_m of radius_m around center. Together with the heading sweep
        this is what tells a flown orbit apart from a vehicle that only turned on the spot."""
        ct = self._home_frame()

        with self._velocity_lock:
            ground_truth_lla = list(self._ground_truth_lla)

        assert len(ground_truth_lla) > 100, \
            "Only {} ground truth samples recorded".format(len(ground_truth_lla))

        distances_m = []
        for latitude_deg, longitude_deg in ground_truth_lla:
            north_m, east_m = ct.local_from_global(latitude_deg, longitude_deg)
            distances_m.append(math.hypot(north_m - center[0], east_m - center[1]))

        min_distance_m = min(distances_m)
        max_distance_m = max(distances_m)
        log("ground truth distance to the orbit center: {:.2f} m to {:.2f} m (radius {} m)"
            .format(min_distance_m, max_distance_m, radius_m))

        assert min_distance_m > radius_m - tolerance_m, \
            "Minimum distance to center {:.2f} m, expected > {} m".format(
                min_distance_m, radius_m - tolerance_m)
        assert max_distance_m < radius_m + tolerance_m, \
            "Maximum distance to center {:.2f} m, expected < {} m".format(
                max_distance_m, radius_m + tolerance_m)

    def check_velocity_error_below(self, mean_tol_m_s: float, max_tol_m_s: float) -> None:
        """Assert on the collected velocity error: the mean absolute error of every axis has to
        stay below mean_tol_m_s and no single sample may be off by more than max_tol_m_s."""
        with self._velocity_lock:
            error_samples = self._error_samples
            error_sum_m_s = list(self._error_sum_m_s)
            error_max_m_s = list(self._error_max_m_s)

        # Without samples the checks below would pass vacuously.
        assert error_samples > 100, "Only {} velocity error samples".format(error_samples)

        for i, axis in enumerate(AXES):
            mean_m_s = error_sum_m_s[i] / error_samples
            log("velocity error {}: mean {:.3f} m/s, max {:.3f} m/s ({} samples)".format(
                axis, mean_m_s, error_max_m_s[i], error_samples))
            assert mean_m_s < mean_tol_m_s, \
                "Mean {} velocity error {:.3f} m/s, expected < {} m/s".format(
                    axis, mean_m_s, mean_tol_m_s)
            assert error_max_m_s[i] < max_tol_m_s, \
                "Max {} velocity error {:.3f} m/s, expected < {} m/s".format(
                    axis, error_max_m_s[i], max_tol_m_s)
