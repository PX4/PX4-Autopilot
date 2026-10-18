"""Helper around MAVSDK to drive PX4 through integration tests.

Timeouts and sleeps are in autopilot (simulation) time once a vehicle is connected, so tests
behave the same at any simulation speed factor.

Telemetry callbacks run on a MAVSDK thread, where a failed assert would only be printed and
swallowed. Checks done in callbacks are therefore collected with check_in_callback() and
reported when the tester is closed.
"""

import datetime
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, List, Optional, Tuple

from mavsdk import ComponentType, Configuration, ConnectionResult, Mavsdk  # type: ignore
from mavsdk.plugins.action import Action, ActionError  # type: ignore
from mavsdk.plugins.events import Events  # type: ignore
from mavsdk.plugins.info import Info, InfoError  # type: ignore
from mavsdk.plugins.mission import Mission, MissionError, MissionItem, MissionPlan  # type: ignore
from mavsdk.plugins.offboard import (  # type: ignore
    Offboard, OffboardError, PositionNedYaw, VelocityNedYaw)
from mavsdk.plugins.param import Param  # type: ignore
from mavsdk.plugins.telemetry import FlightMode, LandedState, Telemetry, VtolState  # type: ignore

from geometry import CoordinateTransformation, Vec3, point_to_line_distance

# Only print the first few callback check failures, they can come at telemetry rate.
MAX_PRINTED_CALLBACK_FAILURES = 10


def log(message: str) -> None:
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    print("[{}|Info ] {}".format(timestamp, message), flush=True)


@dataclass
class MissionOptions:
    leg_length_m: float = 20.0
    relative_altitude_m: float = 10.0
    rtl_at_end: bool = False
    fly_through: bool = False


class HeightSource(Enum):
    BARO = 0
    GPS = 1


class RcLossException(Enum):
    MISSION = 0
    HOLD = 1
    OFFBOARD = 2


class AutopilotTester:

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        self._speed_factor = speed_factor
        self._mavsdk: Any = None
        self._action: Any = None
        self._events: Any = None
        self._info: Any = None
        self._mission: Any = None
        self._offboard: Any = None
        self._param: Any = None
        self._telemetry: Any = None

        self._home: Optional[Tuple[float, float]] = None
        self._check_altitude_handle: Any = None

        self._failures_lock = threading.Lock()
        self._callback_failures: List[str] = []

        self._should_exit = threading.Event()
        self._speed_factor_thread = threading.Thread(target=self._report_speed_factor, daemon=True)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self, url: str) -> None:
        self._mavsdk = Mavsdk(Configuration.create_with_component_type(
            ComponentType.GROUND_STATION))
        result = self._mavsdk.add_any_connection(url)
        assert result == ConnectionResult.SUCCESS, \
            "Connecting to {} failed: {}".format(url, result)

        log("Waiting for system connect")
        system = self._mavsdk.first_autopilot(timeout_s=25.0)
        assert system is not None, "No autopilot found within 25 s"

        self._action = Action(system)
        self._events = Events(system)
        self._info = Info(system)
        self._mission = Mission(system)
        self._offboard = Offboard(system)
        self._param = Param(system)
        self._telemetry = Telemetry(system)

        self._events.subscribe_events(self._print_event)
        self._speed_factor_thread.start()

    def close(self) -> List[str]:
        """Shut down MAVSDK and return the checks that failed in callbacks."""
        self._should_exit.set()
        if self._speed_factor_thread.is_alive():
            self._speed_factor_thread.join()

        if self._mavsdk is not None:
            # Also releases the plugins and their subscriptions.
            self._mavsdk.destroy()
            self._mavsdk = None

        with self._failures_lock:
            failures = list(self._callback_failures)

        if len(failures) > MAX_PRINTED_CALLBACK_FAILURES:
            log("{} checks failed in callbacks in total".format(len(failures)))
        return failures

    # ------------------------------------------------------------------
    # Checks and waiting
    # ------------------------------------------------------------------

    def check_in_callback(self, ok: bool, message: str) -> None:
        """Record a failed check from a MAVSDK callback thread."""
        if ok:
            return
        with self._failures_lock:
            self._callback_failures.append(message)
            count = len(self._callback_failures)
        if count <= MAX_PRINTED_CALLBACK_FAILURES:
            log("Check failed: {}".format(message))

    def _autopilot_time_us(self) -> int:
        if self._telemetry is None:
            return 0
        return int(self._telemetry.attitude_quaternion().timestamp_us)

    def poll_condition_with_timeout(self, condition: Callable[[], bool], timeout_s: float) -> bool:
        """Poll until condition() is true. Return False on timeout.

        Once a vehicle is connected the timeout is in autopilot time. There is deliberately no
        wall-clock limit on top: a busy host can slow PX4 down, and the runner has its own
        per-test timeout.
        """
        check_interval_s = min(timeout_s / 100.0, 0.02)
        start_us = self._autopilot_time_us()

        if start_us != 0:
            while not condition():
                time.sleep(check_interval_s)
                elapsed_s = (self._autopilot_time_us() - start_us) / 1e6
                if elapsed_s > timeout_s:
                    log("Timeout, connected to vehicle but waiting for test for {:.1f} seconds"
                        .format(elapsed_s))
                    return False
        else:
            start = time.monotonic()
            while not condition():
                time.sleep(check_interval_s)
                elapsed_s = time.monotonic() - start
                if elapsed_s > timeout_s:
                    log("Timeout, waiting for the vehicle for {:.1f} seconds".format(elapsed_s))
                    return False
        return True

    def sleep_for(self, duration_s: float) -> None:
        """Sleep in autopilot time (host time if nothing is connected)."""
        start_us = self._autopilot_time_us()
        if start_us == 0:
            time.sleep(duration_s)
            return

        while (self._autopilot_time_us() - start_us) / 1e6 < duration_s:
            # Often enough not to have PX4 time out on us.
            time.sleep(0.001)

    def wait_until_ready(self) -> None:
        log("Waiting for system to be ready (system health ok & able to arm)")
        assert self.poll_condition_with_timeout(
            lambda: bool(self._telemetry.health_all_ok()), 30), "System health not ok"

        # health_all_ok() can be true before the global position is considered valid (MAVSDK
        # issue #1852). Waiting for is_armable covers that, since arming needs a valid global
        # position.
        assert self.poll_condition_with_timeout(
            lambda: bool(self._telemetry.health().is_armable), 45), "System not armable"

    def wait_until_disarmed(self, timeout_s: float = 60) -> None:
        assert self.poll_condition_with_timeout(
            lambda: not self._telemetry.armed(), timeout_s), \
            "Not disarmed within {} s".format(timeout_s)

    def wait_until_hovering(self) -> None:
        self.wait_for_landed_state(LandedState.IN_AIR, 45)

    def wait_until_altitude(self, rel_altitude_m: float, timeout_s: float,
                            delta: float = 0.5) -> None:
        def reached() -> bool:
            down_m = float(self._telemetry.position_velocity_ned().position.down_m)
            return abs(rel_altitude_m + down_m) <= delta

        assert self.poll_condition_with_timeout(reached, timeout_s), \
            "Altitude {} m (+-{} m) not reached within {} s".format(
                rel_altitude_m, delta, timeout_s)

    def wait_until_multicopter(self, timeout_s: float) -> None:
        assert self.poll_condition_with_timeout(
            lambda: self._telemetry.vtol_state() == VtolState.MC, timeout_s), \
            "Not in multicopter mode within {} s".format(timeout_s)

    def wait_until_fixedwing(self, timeout_s: float) -> None:
        assert self.poll_condition_with_timeout(
            lambda: self._telemetry.vtol_state() == VtolState.FW, timeout_s), \
            "Not in fixed-wing mode within {} s".format(timeout_s)

    def wait_until_speed_lower_than(self, speed_m_s: float, timeout_s: float) -> None:
        def slow_enough() -> bool:
            vel = self._telemetry.position_velocity_ned().velocity
            speed = math.sqrt(vel.north_m_s ** 2 + vel.east_m_s ** 2 + vel.down_m_s ** 2)
            return bool(speed < speed_m_s)

        assert self.poll_condition_with_timeout(slow_enough, timeout_s), \
            "Speed not below {} m/s within {} s".format(speed_m_s, timeout_s)

    def wait_for_flight_mode(self, flight_mode: FlightMode, timeout_s: float) -> None:
        assert self.poll_condition_with_timeout(
            lambda: self._telemetry.flight_mode() == flight_mode, timeout_s), \
            "Flight mode {} not reached within {} s".format(flight_mode, timeout_s)

    def wait_for_landed_state(self, landed_state: LandedState, timeout_s: float) -> None:
        assert self.poll_condition_with_timeout(
            lambda: self._telemetry.landed_state() == landed_state, timeout_s), \
            "Landed state {} not reached within {} s".format(landed_state, timeout_s)

    # ------------------------------------------------------------------
    # Parameters and settings
    # ------------------------------------------------------------------

    def set_param_int(self, name: str, value: int) -> None:
        self._param.set_param_int(name, value)

    def set_param_float(self, name: str, value: float) -> None:
        self._param.set_param_float(name, value)

    def set_takeoff_altitude(self, altitude_m: float) -> None:
        self._action.set_takeoff_altitude(altitude_m)
        actual = float(self._action.get_takeoff_altitude())
        assert math.isclose(actual, altitude_m, rel_tol=1e-5), \
            "Takeoff altitude is {}, expected {}".format(actual, altitude_m)

    def set_rtl_altitude(self, altitude_m: float) -> None:
        self._action.set_return_to_launch_altitude(altitude_m)
        actual = float(self._action.get_return_to_launch_altitude())
        assert math.isclose(actual, altitude_m, rel_tol=1e-5), \
            "RTL altitude is {}, expected {}".format(actual, altitude_m)

    def set_height_source(self, height_source: HeightSource) -> None:
        self.set_param_int("EKF2_HGT_REF", height_source.value)

    def set_rc_loss_exception(self, exception: RcLossException) -> None:
        self.set_param_int("COM_RCL_EXCEPT", 1 << exception.value)

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def arm(self) -> None:
        # Even after wait_until_ready() passes, is_armable can briefly flicker (e.g. the
        # heading reference dropping out during startup), so the arm command gets denied.
        # Retry until it is accepted.
        def try_arm() -> bool:
            try:
                self._action.arm()
                return True
            except ActionError as e:
                log("Arming failed: {}".format(e))
                return False

        assert self.poll_condition_with_timeout(try_arm, 30), "Could not arm within 30 s"

    def takeoff(self) -> None:
        self._action.takeoff()

    def land(self) -> None:
        self._action.land()

    def execute_land(self) -> None:
        self._action.land()

    def execute_rtl(self) -> None:
        self._action.return_to_launch()

    def transition_to_fixedwing(self) -> None:
        self._action.transition_to_fixedwing()

    def transition_to_multicopter(self) -> None:
        self._action.transition_to_multicopter()

    # ------------------------------------------------------------------
    # Position and home
    # ------------------------------------------------------------------

    def request_ground_truth(self) -> None:
        self._telemetry.set_rate_ground_truth(15)

    def store_home(self) -> None:
        self.request_ground_truth()
        log("Waiting to get home position")

        def got_home() -> bool:
            gt = self._telemetry.ground_truth()
            if math.isfinite(gt.latitude_deg) and math.isfinite(gt.longitude_deg):
                self._home = (float(gt.latitude_deg), float(gt.longitude_deg))
                return True
            return False

        assert self.poll_condition_with_timeout(got_home, 10), "No ground truth position received"

    def _home_or_fail(self) -> Tuple[float, float]:
        assert self._home is not None, "store_home() was not called"
        return self._home

    def check_home_within(self, acceptance_radius_m: float) -> None:
        distance_m = self._ground_truth_horizontal_distance_to(*self._home_or_fail())
        assert distance_m < acceptance_radius_m, \
            "Distance to home {:.2f} m, expected < {} m".format(distance_m, acceptance_radius_m)

    def check_home_not_within(self, min_distance_m: float) -> None:
        distance_m = self._ground_truth_horizontal_distance_to(*self._home_or_fail())
        assert distance_m > min_distance_m, \
            "Distance to home {:.2f} m, expected > {} m".format(distance_m, min_distance_m)

    def _ground_truth_horizontal_distance_to(self, latitude_deg: float,
                                             longitude_deg: float) -> float:
        assert math.isfinite(latitude_deg) and math.isfinite(longitude_deg)
        current = self._telemetry.ground_truth()
        assert math.isfinite(current.latitude_deg) and math.isfinite(current.longitude_deg), \
            "No ground truth position"

        ct = CoordinateTransformation(latitude_deg, longitude_deg)
        north_m, east_m = ct.local_from_global(current.latitude_deg, current.longitude_deg)
        distance_m = math.hypot(north_m, east_m)
        log("Target: {:.7f}, {:.7f}, current: {:.7f}, {:.7f}, distance: {:.2f} m".format(
            latitude_deg, longitude_deg, current.latitude_deg, current.longitude_deg,
            distance_m))
        return distance_m

    def get_current_position_ned(self) -> Vec3:
        position = self._telemetry.position_velocity_ned().position
        return float(position.north_m), float(position.east_m), float(position.down_m)

    def get_coordinate_transformation(self) -> CoordinateTransformation:
        home = self._telemetry.home()
        assert math.isfinite(home.latitude_deg) and math.isfinite(home.longitude_deg), \
            "No home position"
        return CoordinateTransformation(home.latitude_deg, home.longitude_deg)

    def check_current_altitude(self, target_rel_altitude_m: float,
                               max_distance_m: float = 1.5) -> None:
        altitude_m = self._telemetry.position().relative_altitude_m
        assert abs(altitude_m - target_rel_altitude_m) <= max_distance_m, \
            "Relative altitude {:.2f} m, expected {} m (+-{} m)".format(
                altitude_m, target_rel_altitude_m, max_distance_m)

    def start_checking_altitude(self, max_deviation_m: float) -> None:
        """Fail the test if the altitude deviates from the current one by more than
        max_deviation_m, until stop_checking_altitude()."""
        target_down_m = self.get_current_position_ned()[2]

        def check(position_velocity_ned: Any, _user_data: Any) -> None:
            deviation_m = abs(target_down_m - position_velocity_ned.position.down_m)
            self.check_in_callback(
                deviation_m <= max_deviation_m,
                "Altitude deviation {:.3f} m exceeds {} m".format(deviation_m, max_deviation_m))

        self._check_altitude_handle = self._telemetry.subscribe_position_velocity_ned(check)

    def stop_checking_altitude(self) -> None:
        if self._check_altitude_handle is not None:
            self._telemetry.unsubscribe_position_velocity_ned(self._check_altitude_handle)
            self._check_altitude_handle = None

    # ------------------------------------------------------------------
    # Missions
    # ------------------------------------------------------------------

    @staticmethod
    def _create_mission_item(north_m: float, east_m: float, options: MissionOptions,
                             ct: CoordinateTransformation) -> Any:
        latitude_deg, longitude_deg = ct.global_from_local(north_m, east_m)
        # All fields need to be set, the Python defaults are None.
        nan = float("nan")
        return MissionItem(
            latitude_deg=latitude_deg,
            longitude_deg=longitude_deg,
            relative_altitude_m=options.relative_altitude_m,
            speed_m_s=nan,
            is_fly_through=options.fly_through,
            gimbal_pitch_deg=nan,
            gimbal_yaw_deg=nan,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=nan,
            camera_photo_interval_s=1.0,
            acceptance_radius_m=nan,
            yaw_deg=nan,
            camera_photo_distance_m=nan,
            vehicle_action=MissionItem.VehicleAction.NONE)

    def _upload_mission(self, local_positions: List[Tuple[float, float]],
                        options: MissionOptions) -> None:
        ct = self.get_coordinate_transformation()
        items = [self._create_mission_item(north_m, east_m, options, ct)
                 for north_m, east_m in local_positions]

        self._mission.set_return_to_launch_after_mission(options.rtl_at_end)
        self._mission.upload_mission(MissionPlan(mission_items=items))
        # PX4 needs time to realize that it now has a mission available.
        self.sleep_for(1)

    def prepare_square_mission(self, options: MissionOptions) -> None:
        leg = options.leg_length_m
        self._upload_mission([(leg, 0.0), (leg, leg), (0.0, leg)], options)

    def prepare_straight_mission(self, options: MissionOptions) -> None:
        leg = options.leg_length_m
        self._upload_mission([(i * leg, 0.0) for i in range(5)], options)

    def _try_start_mission(self) -> bool:
        try:
            self._mission.start_mission()
            return True
        except MissionError as e:
            log("Starting mission failed: {}".format(e))
            return False

    def _is_mission_finished(self) -> bool:
        try:
            return bool(self._mission.is_mission_finished())
        except MissionError:
            return False

    def execute_mission(self) -> None:
        assert self.poll_condition_with_timeout(self._try_start_mission, 3), \
            "Could not start mission"
        self.wait_for_mission_finished(500)

    def start_and_wait_for_mission_sequence(self, sequence_number: int) -> None:
        self._mission.start_mission()
        assert self.poll_condition_with_timeout(
            lambda: self._mission.mission_progress().current >= sequence_number, 60), \
            "Mission item {} not reached within 60 s".format(sequence_number)

    def wait_for_mission_finished(self, timeout_s: float) -> None:
        assert self.poll_condition_with_timeout(self._is_mission_finished, timeout_s), \
            "Mission not finished within {} s".format(timeout_s)

    def check_tracks_mission(self, corridor_radius_m: float = 1.5) -> None:
        """Fail the test if the vehicle leaves the corridor around the line between the
        previous and the current mission item."""
        items = self._mission.download_mission().mission_items
        ct = self.get_coordinate_transformation()

        def local(item: Any) -> Vec3:
            north_m, east_m = ct.local_from_global(item.latitude_deg, item.longitude_deg)
            return north_m, east_m, -item.relative_altitude_m

        waypoints = [local(item) for item in items]

        def check(position_velocity_ned: Any, _user_data: Any) -> None:
            progress = self._mission.mission_progress()
            if not 0 < progress.current < progress.total:
                return
            position = position_velocity_ned.position
            distance_m = point_to_line_distance(
                (position.north_m, position.east_m, position.down_m),
                waypoints[progress.current - 1],
                waypoints[progress.current])
            self.check_in_callback(
                distance_m < corridor_radius_m,
                "Distance to mission track {:.2f} m exceeds {} m".format(
                    distance_m, corridor_radius_m))

        self._telemetry.set_rate_position_velocity_ned(5)
        self._telemetry.subscribe_position_velocity_ned(check)

    # ------------------------------------------------------------------
    # Offboard
    # ------------------------------------------------------------------

    def _start_offboard_with_retry(self, resend_setpoint: Callable[[], None]) -> None:
        # MAVSDK's offboard watchdog resets the setpoint state when a heartbeat without
        # offboard mode arrives before the first start(). At high speed factors one can slip
        # in between set_*() and start(), so start() fails with NoSetpointSet. Re-send and
        # retry.
        for _ in range(10):
            try:
                self._offboard.start()
                return
            except OffboardError as e:
                error = e
            resend_setpoint()
            time.sleep(0.02)
        raise error

    def offboard_goto(self, target: PositionNedYaw, acceptance_radius_m: float = 0.3,
                      timeout_s: float = 60) -> None:
        self._offboard.set_position_ned(target)
        self._start_offboard_with_retry(lambda: self._offboard.set_position_ned(target))

        def close_enough() -> bool:
            position = self._telemetry.position_velocity_ned().position
            distance_m = math.sqrt((position.north_m - target.north_m) ** 2
                                   + (position.east_m - target.east_m) ** 2
                                   + (position.down_m - target.down_m) ** 2)
            return bool(distance_m < acceptance_radius_m)

        assert self.poll_condition_with_timeout(close_enough, timeout_s), \
            "Offboard target not reached within {} s".format(timeout_s)
        log("Target position reached")

    def offboard_land(self) -> None:
        self._offboard.set_velocity_ned(VelocityNedYaw(
            north_m_s=0.0, east_m_s=0.0, down_m_s=1.0, yaw_deg=0.0))

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _print_event(event: Any, _user_data: Any) -> None:
        print("[{}] {}".format(event.log_level, event.message), flush=True)
        if event.description:
            print("    Description: {}".format(event.description), flush=True)
        print("    Event name: {}/{}".format(event.event_namespace, event.event_name), flush=True)

    def _report_speed_factor(self) -> None:
        while not self._should_exit.wait(1.0):
            try:
                current = float(self._info.get_speed_factor())
            except InfoError:
                continue
            message = "Current speed factor: {:.2f}".format(current)
            if self._speed_factor is not None:
                message += " (set: {})".format(self._speed_factor)
            print(message, flush=True)
