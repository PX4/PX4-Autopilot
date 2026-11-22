"""AutopilotTester for fixed-wing loiter, reposition, Hold and figure-eight behaviour.

Positions are home-relative local (north, east) coordinates, converted to global setpoints
internally. The loiter geometry can only be observed indirectly (no MAVLink message exposes the
internal position setpoint), so the checks are trajectory based: they assert where the vehicle
actually flies over a time window.
"""

import math
from typing import Any, List, Optional, Tuple

from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import AutopilotTester, log

MAV_CMD_DO_ORBIT = 34
MAV_CMD_DO_FIGURE_EIGHT = 35
MAV_CMD_DO_REPOSITION = 192

# MAV_DO_REPOSITION_FLAGS_CHANGE_MODE: switch into Hold if not already there.
REPOSITION_FLAG_CHANGE_MODE = 1.0

# INT32_MAX in x/y of COMMAND_INT means "not set", i.e. keep the current position.
COORDINATE_NOT_SET = 2**31 - 1

Local2 = Tuple[float, float]


def horizontal_distance(sample: Any, point: Local2) -> float:
    return math.hypot(sample.position.north_m - point[0], sample.position.east_m - point[1])


def relative_altitude(sample: Any) -> float:
    return float(-sample.position.down_m)


class AutopilotTesterLoiter(AutopilotTester):

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def enter_hold_and_wait(self, timeout_s: float = 10) -> None:
        self.hold()
        self.wait_for_flight_mode(FlightMode.HOLD, timeout_s)

    def _global_int(self, point: Local2) -> Tuple[int, int]:
        latitude_deg, longitude_deg = self.get_coordinate_transformation().global_from_local(
            *point)
        return int(latitude_deg * 1e7), int(longitude_deg * 1e7)

    def command_orbit(self, center: Local2, radius_m: float, rel_alt_m: float) -> None:
        """Fixed-wing orbit around center. The radius sign is the direction (positive is
        clockwise). Needs Hold for the setpoint to be picked up."""
        x, y = self._global_int(center)
        nan = float("nan")
        self.send_command_int(MAV_CMD_DO_ORBIT, param1=radius_m, param2=nan, param3=nan,
                              param4=nan, x=x, y=y,
                              z=self.home_absolute_altitude_m() + rel_alt_m)

    def command_figure_eight(self, center: Local2, major_axis_m: float, minor_axis_m: float,
                             rel_alt_m: float, orientation_rad: float = 0.0) -> None:
        """Figure eight around center. The major axis sign is the direction."""
        x, y = self._global_int(center)
        self.send_command_int(MAV_CMD_DO_FIGURE_EIGHT, param1=major_axis_m, param2=minor_axis_m,
                              param3=float("nan"), param4=orientation_rad, x=x, y=y,
                              z=self.home_absolute_altitude_m() + rel_alt_m)

    def _reposition(self, x: int, y: int, z: float) -> None:
        # Default ground speed, switch to Hold, yaw unchanged.
        self.send_command_int(MAV_CMD_DO_REPOSITION, param1=-1.0,
                              param2=REPOSITION_FLAG_CHANGE_MODE, param3=0.0,
                              param4=float("nan"), x=x, y=y, z=z)

    def command_reposition(self, target: Local2, rel_alt_m: float) -> None:
        """Plain reposition: fly straight toward target."""
        x, y = self._global_int(target)
        self._reposition(x, y, self.home_absolute_altitude_m() + rel_alt_m)

    def command_hold_here(self) -> None:
        """Pause/Hold reposition: position and altitude not set."""
        self._reposition(COORDINATE_NOT_SET, COORDINATE_NOT_SET, float("nan"))

    def command_reposition_altitude(self, rel_alt_m: float) -> None:
        """Reposition that only changes the altitude."""
        self._reposition(COORDINATE_NOT_SET, COORDINATE_NOT_SET,
                         self.home_absolute_altitude_m() + rel_alt_m)

    # ------------------------------------------------------------------
    # Waits
    # ------------------------------------------------------------------

    def wait_until_on_loiter(self, center: Local2, radius_m: float, radius_tol_m: float,
                             settle_s: float, timeout_s: float) -> None:
        """Wait until the vehicle reaches the radial band around center, then settle_s more.

        Flying out to the circle only crosses the band once and then stays on it, so the first
        entry marks establishment.
        """
        self.wait_for_position_ned(
            lambda s: abs(horizontal_distance(s, center) - radius_m) <= radius_tol_m, timeout_s)
        self.sleep_for(settle_s)
        log("Established on loiter")

    def wait_until_relative_altitude_above(self, threshold_m: float, timeout_s: float) -> None:
        # A threshold rather than a band, so it cannot be skipped between samples at a high
        # speed factor.
        self.wait_for_position_ned(lambda s: relative_altitude(s) >= threshold_m, timeout_s)

    # ------------------------------------------------------------------
    # Trajectory checks
    # ------------------------------------------------------------------

    def check_stays_on_loiter(self, center: Local2, radius_m: float, radius_tol_m: float,
                              duration_s: float) -> None:
        def check(sample: Any) -> Optional[str]:
            distance_m = horizontal_distance(sample, center)
            if abs(distance_m - radius_m) > radius_tol_m:
                return "Distance to loiter center {:.1f} m, expected {} +- {} m".format(
                    distance_m, radius_m, radius_tol_m)
            return None

        self.check_position_ned_for(check, duration_s)

    def check_stays_within(self, center: Local2, max_radius_m: float, duration_s: float) -> None:
        def check(sample: Any) -> Optional[str]:
            distance_m = horizontal_distance(sample, center)
            if distance_m > max_radius_m:
                return "Distance to {} is {:.1f} m, expected <= {} m".format(
                    center, distance_m, max_radius_m)
            return None

        self.check_position_ned_for(check, duration_s)

    def check_never_reaches(self, point: Local2, min_distance_m: float,
                            duration_s: float) -> None:
        def check(sample: Any) -> Optional[str]:
            distance_m = horizontal_distance(sample, point)
            if distance_m < min_distance_m:
                return "Distance to {} is {:.1f} m, expected >= {} m".format(
                    point, distance_m, min_distance_m)
            return None

        self.check_position_ned_for(check, duration_s)

    def check_holds_altitude(self, rel_alt_m: float, tol_m: float, duration_s: float) -> None:
        def check(sample: Any) -> Optional[str]:
            altitude_m = relative_altitude(sample)
            if abs(altitude_m - rel_alt_m) > tol_m:
                return "Relative altitude {:.1f} m, expected {:.1f} +- {} m".format(
                    altitude_m, rel_alt_m, tol_m)
            return None

        self.check_position_ned_for(check, duration_s)

    def check_climbs_on_loiter(self, center: Local2, radius_m: float, radius_tol_m: float,
                               target_rel_alt_m: float, alt_tol_m: float,
                               timeout_s: float) -> None:
        """Check that the vehicle climbs to the target altitude without ever leaving the radial
        band around center."""
        def done(sample: Any) -> bool:
            distance_m = horizontal_distance(sample, center)
            self.check_in_callback(
                abs(distance_m - radius_m) <= radius_tol_m,
                "Left the loiter while climbing: distance to center {:.1f} m, "
                "expected {} +- {} m".format(distance_m, radius_m, radius_tol_m))
            return abs(relative_altitude(sample) - target_rel_alt_m) <= alt_tol_m

        self.wait_for_position_ned(done, timeout_s)
        self._assert_no_callback_failures()
        log("Climbed to target altitude while staying on loiter")

    def check_tracks_figure_eight(self, center: Local2, major_axis_m: float,
                                  minor_axis_m: float, orientation_rad: float,
                                  corridor_radius_m: float, timeout_s: float) -> None:
        """Check that the vehicle passes the points of interest of the figure eight in order:
        center, one lobe, center, other lobe, center."""
        cos_o = math.cos(orientation_rad)
        sin_o = math.sin(orientation_rad)
        major = abs(major_axis_m)
        minor = minor_axis_m

        def point(along: float, across: float) -> Local2:
            return (center[0] + cos_o * along - sin_o * across,
                    center[1] + sin_o * along + cos_o * across)

        points: List[Local2] = [
            center,
            point(major - minor, -minor), point(major, 0.0), point(major - minor, minor),
            point(-major + minor, -minor), point(-major, 0.0), point(-major + minor, minor),
        ]
        order = [0, 1, 2, 3, 0, 4, 5, 6, 0] if major_axis_m > 0 else [0, 3, 2, 1, 0, 6, 5, 4, 0]

        index = 0
        out_of_order: List[int] = []

        def done(sample: Any) -> bool:
            nonlocal index
            close = next((i for i, p in enumerate(points)
                          if abs(sample.position.north_m - p[0]) < corridor_radius_m
                          and abs(sample.position.east_m - p[1]) < corridor_radius_m), None)
            if close is not None and close != order[index]:
                if close == order[index + 1]:
                    index += 1
                elif index > 0:
                    # Out of order, but only once past the first center point.
                    out_of_order.append(close)
                    return True
            return index + 1 == len(order)

        self.wait_for_position_ned(done, timeout_s)
        assert not out_of_order, "Reached figure eight point {} out of order".format(
            out_of_order[0])

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    def current_relative_altitude(self) -> float:
        # NED down, to match the altitude used by the trajectory checks.
        return relative_altitude(self._telemetry.position_velocity_ned())

    def current_local_position(self) -> Local2:
        position = self._telemetry.position_velocity_ned().position
        return float(position.north_m), float(position.east_m)

    def default_loiter_radius(self) -> float:
        return self.get_param_float("NAV_LOITER_RAD")

    def fw_altitude_acceptance_radius(self) -> float:
        return self.get_param_float("NAV_FW_ALT_RAD")
