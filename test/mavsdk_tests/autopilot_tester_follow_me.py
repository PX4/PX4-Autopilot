"""AutopilotTester for follow-me (follow target) tests."""

import math
import random
from typing import Any, Optional

from mavsdk.plugins.follow_me import Config, FollowMe, TargetLocation  # type: ignore

from autopilot_tester import AutopilotTester, log
from geometry import CoordinateTransformation, Vec3, diff, norm


def follow_me_config(follow_height_m: float = 8.0, follow_distance_m: float = 8.0,
                     follow_angle_deg: float = 180.0,
                     max_tangential_vel_m_s: float = 8.0) -> Any:
    # All fields need to be set, the Python defaults are None. These are the C++ defaults.
    return Config(follow_height_m=follow_height_m, follow_distance_m=follow_distance_m,
                  responsiveness=0.1, altitude_mode=Config.FollowAltitudeMode.CONSTANT,
                  max_tangential_vel_m_s=max_tangential_vel_m_s,
                  follow_angle_deg=follow_angle_deg)


def target_location(latitude_deg: float = math.nan, longitude_deg: float = math.nan,
                    absolute_altitude_m: float = math.nan, velocity: Vec3 = (
                        math.nan, math.nan, math.nan)) -> Any:
    return TargetLocation(latitude_deg=latitude_deg, longitude_deg=longitude_deg,
                          absolute_altitude_m=absolute_altitude_m, velocity_x_m_s=velocity[0],
                          velocity_y_m_s=velocity[1], velocity_z_m_s=velocity[2])


class FollowTargetSimulator:
    """A target moving north in a straight line at 2 m/s, with deterministic noise that changes
    with every update()."""

    SPEED_M_S = 2.0
    POSITION_NOISE_STD_M = 1.0
    VELOCITY_NOISE_STD_M_S = 0.1

    def __init__(self, initial_position_ned: Vec3, home_latitude_deg: float,
                 home_longitude_deg: float, home_altitude_m: float) -> None:
        self._position_ned = list(initial_position_ned)
        self._velocity_ned = [0.0, 0.0, 0.0]
        self._ct = CoordinateTransformation(home_latitude_deg, home_longitude_deg)
        self._home_altitude_m = home_altitude_m
        self._update_count = 0

    def update(self, dt_s: float) -> None:
        self._velocity_ned = [self.SPEED_M_S, 0.0, 0.0]
        for i in range(3):
            self._position_ned[i] += self._velocity_ned[i] * dt_s
        self._update_count += 1

    def _noise(self, std: float) -> Vec3:
        rng = random.Random(self._update_count)
        return rng.gauss(0.0, std), rng.gauss(0.0, std), rng.gauss(0.0, std)

    def position_ned(self) -> Vec3:
        return self._position_ned[0], self._position_ned[1], self._position_ned[2]

    def global_position(self, add_noise: bool) -> Vec3:
        """(latitude_deg, longitude_deg, absolute_altitude_m)"""
        north_m, east_m, down_m = self.position_ned()
        if add_noise:
            noise = self._noise(self.POSITION_NOISE_STD_M)
            north_m, east_m, down_m = north_m + noise[0], east_m + noise[1], down_m + noise[2]
        latitude_deg, longitude_deg = self._ct.global_from_local(north_m, east_m)
        # Same as the C++ test: the down offset is added to the home altitude.
        return latitude_deg, longitude_deg, down_m + self._home_altitude_m

    def velocity_ned_noisy(self) -> Vec3:
        noise = self._noise(self.VELOCITY_NOISE_STD_M_S)
        return (self._velocity_ned[0] + noise[0], self._velocity_ned[1] + noise[1],
                self._velocity_ned[2] + noise[2])


class AutopilotTesterFollowMe(AutopilotTester):

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        super().__init__(speed_factor)
        self._follow_me: Any = None

    def connect(self, url: str) -> None:
        super().connect(url)
        self._follow_me = FollowMe(self._system)

    def _target_simulator(self) -> FollowTargetSimulator:
        # Start the target on the drone's position.
        return FollowTargetSimulator(self.get_current_position_ned(), *self._home_or_fail(),
                                     self.home_absolute_altitude_m())

    def _start_follow_me(self) -> None:
        self._follow_me.start()
        # Target locations are only accepted once MAVSDK sees the vehicle in follow-me mode.
        assert self.poll_condition_with_timeout(
            lambda: bool(self._follow_me.is_active()), 5), "Follow-me not active within 5 s"

    def _check(self, ok: bool, message: str) -> None:
        # Record and continue, so one run shows everything that's off.
        self.check_in_callback(ok, message)

    def _check_follow_position(self, config: Any, drone: Vec3, target: Vec3,
                               tolerance_m: float) -> None:
        """Assumes that the target travels north. The follow angle is relative to the target's
        course."""
        offset_north_m = drone[0] - target[0]
        offset_east_m = drone[1] - target[1]
        angle_rad = math.radians(config.follow_angle_deg)
        expected_north_m = config.follow_distance_m * math.cos(angle_rad)
        expected_east_m = config.follow_distance_m * math.sin(angle_rad)
        self._check(abs(offset_north_m - expected_north_m) < tolerance_m and
                    abs(offset_east_m - expected_east_m) < tolerance_m,
                    "Follow angle {}: offset to target ({:.1f}, {:.1f}) m, expected "
                    "({:.1f}, {:.1f}) +- {} m".format(
                        config.follow_angle_deg, offset_north_m, offset_east_m,
                        expected_north_m, expected_east_m, tolerance_m))

    def straight_line_test(self, stream_velocity: bool) -> None:
        """Move the target north and check that the drone follows from behind, front, front
        right and front left."""
        update_rate_hz = 1
        tolerance_m = 11.0
        follow_height_m = 10.0

        target = self._target_simulator()
        config = follow_me_config(follow_height_m=follow_height_m, follow_distance_m=8.0,
                                  follow_angle_deg=0.0,
                                  # Fast enough tangential velocity to be responsive.
                                  max_tangential_vel_m_s=15.0)

        # Allow some time for the mode switch.
        self.sleep_for(1)
        self._start_follow_me()

        # (sample index, follow angle) at which the follow angle is changed. The drone then gets
        # 10 samples to reach steady state, after which positions are checked until the next
        # change.
        # The target starts moving after the first change.
        angle_changes = {5: 180.0, 20: 0.0, 35: 45.0, 55: -45.0}
        first_change = min(angle_changes)
        check_from = first_change + 10

        for i in range(75 * update_rate_hz):
            target_position = target.position_ned()
            drone_position = self.get_current_position_ned()

            if i in angle_changes:
                config.follow_angle_deg = angle_changes[i]
                self._follow_me.set_config(config)
                log("Follow angle {} deg".format(config.follow_angle_deg))
                check_from = i + 10

            if i > first_change:
                target.update(1.0 / update_rate_hz)

            if i >= check_from:
                altitude_m = self._telemetry.position().relative_altitude_m
                self._check(abs(altitude_m - follow_height_m) <= 1.5,
                            "Relative altitude {:.1f} m, expected {} m".format(
                                altitude_m, follow_height_m))
                distance_m = norm(diff(target_position, drone_position))
                self._check(abs(distance_m - config.follow_distance_m) <= tolerance_m,
                            "Distance to target {:.1f} m, expected {} +- {} m".format(
                                distance_m, config.follow_distance_m, tolerance_m))
                self._check_follow_position(config, drone_position, target_position,
                                            tolerance_m)

            latitude_deg, longitude_deg, altitude_m = target.global_position(add_noise=True)
            velocity = target.velocity_ned_noisy() if stream_velocity \
                else (math.nan, math.nan, math.nan)
            self._follow_me.set_target_location(
                target_location(latitude_deg, longitude_deg, altitude_m, velocity))

            self.sleep_for(1.0 / update_rate_hz)

        self._follow_me.stop()
        self._assert_no_callback_failures()

    def stream_velocity_only(self) -> None:
        """Stream target velocity without a position. Follow-me needs at least one position to
        integrate the velocity, so the drone must not move."""
        position_tolerance_m = 4.0

        self._follow_me.set_config(follow_me_config(follow_angle_deg=180.0))
        self.sleep_for(1)
        self._start_follow_me()

        initial_position = self.get_current_position_ned()

        for _ in range(15):
            self._follow_me.set_target_location(target_location(velocity=(2.0, 1.0, 0.5)))
            self.sleep_for(1)

        distance_m = norm(diff(initial_position, self.get_current_position_ned()))
        assert distance_m < position_tolerance_m, \
            "Moved {:.1f} m with velocity only, expected < {} m".format(
                distance_m, position_tolerance_m)

    def rc_adjustment_test(self) -> None:
        """Adjust follow height (throttle), distance (pitch) and angle (roll) with RC sticks."""
        rate_hz = 50
        follow_height_m = 10.0
        follow_angle_deg = 0.0
        follow_distance_m = 10.0
        max_tangential_vel_m_s = 8.0

        # Adjustment speeds at full deflection, from FlightTaskAutoFollowTarget.hpp. The angle
        # adjustment is additionally limited to max_tangential_vel_m_s / follow distance.
        follow_distance_adjust_speed_m_s = 2.0
        follow_height_adjust_speed_m_s = 1.5
        follow_angle_adjust_speed_rad_s = 1.5

        target = self._target_simulator()
        self._follow_me.set_config(follow_me_config(
            follow_height_m=follow_height_m, follow_distance_m=follow_distance_m,
            follow_angle_deg=follow_angle_deg, max_tangential_vel_m_s=max_tangential_vel_m_s))

        # A stationary target where the drone is now.
        stationary_target = target_location(*target.global_position(add_noise=False))
        target_position = self.get_current_position_ned()

        self._start_follow_me()

        def sticks(x: float, y: float, z: float) -> None:
            self._manual_control.set_manual_control_input(x, y, z, 0.0)

        # Each check compares against the state right before its stick input, since the drone
        # does not necessarily sit exactly at the configured setting.
        height_before_m = 0.0
        distance_before_m = 0.0
        angle_before_rad = 0.0

        for i in range(60 * rate_hz + 1):
            self._follow_me.set_target_location(stationary_target)
            drone = self.get_current_position_ned()
            t_s = i / rate_hz
            height_m = -drone[2]
            distance_m = math.hypot(drone[0] - target_position[0], drone[1] - target_position[1])
            angle_rad = math.atan2(drone[1] - target_position[1], drone[0] - target_position[0])

            if t_s < 5:
                # Give the drone time to reach its initial follow position (in front).
                sticks(0.0, 0.0, 0.5)
            elif i == 5 * rate_hz:
                height_before_m = height_m
                log("Adjusting follow height")
            elif t_s < 8:
                sticks(0.0, 0.0, 1.0)  # throttle up
            elif t_s < 10:
                sticks(0.0, 0.0, 0.5)
            elif i == 10 * rate_hz:
                # At least half of the expected adjustment.
                expected_m = follow_height_adjust_speed_m_s * 3.0 * 0.5
                self._check(height_m - height_before_m > expected_m,
                            "Follow height went up {:.1f} m, expected > {:.1f} m".format(
                                height_m - height_before_m, expected_m))
                distance_before_m = distance_m
                log("Adjusting follow distance")
            elif t_s < 13:
                sticks(1.0, 0.0, 0.5)  # pitch forward
            elif t_s < 15:
                sticks(0.0, 0.0, 0.5)
            elif i == 15 * rate_hz:
                # PX4 only grows the distance setpoint while the drone is within 1 m of it, and the
                # position controller closes 1 m at ~0.75 m/s. So the distance only changes at
                # about 40% of the nominal speed: expect a quarter of the nominal change.
                expected_m = follow_distance_adjust_speed_m_s * 3.0 * 0.25
                self._check(distance_m - distance_before_m > expected_m,
                            "Follow distance went up {:.1f} m to {:.1f} m, expected > {:.1f} m"
                            .format(distance_m - distance_before_m, distance_m, expected_m))
                angle_before_rad = angle_rad
                log("Adjusting follow angle")
            elif t_s < 18:
                sticks(0.0, 1.0, 0.5)  # roll right, follow angle increases clockwise
            elif t_s < 20:
                sticks(0.0, 0.0, 0.5)
            elif i == 20 * rate_hz:
                angle_change_rad = math.remainder(angle_rad - angle_before_rad, 2 * math.pi)
                angle_speed_rad_s = min(follow_angle_adjust_speed_rad_s,
                                        max_tangential_vel_m_s / distance_m)
                expected_rad = angle_speed_rad_s * 3.0 * 0.5
                self._check(angle_change_rad > expected_rad,
                            "Follow angle went up {:.2f} rad, expected > {:.2f} rad".format(
                                angle_change_rad, expected_rad))
            else:
                # Keep manual control alive, otherwise PX4 fails safe after COM_RC_LOSS_T.
                sticks(0.0, 0.0, 0.5)

            self.sleep_for(1.0 / rate_hz)

        self._assert_no_callback_failures()
