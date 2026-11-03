"""Helper around MAVSDK to drive PX4 through integration tests.

Timeouts and sleeps are in autopilot (simulation) time once a vehicle is connected, so tests
behave the same at any simulation speed factor.

Telemetry callbacks run on a MAVSDK thread, where a failed assert would only be printed and
swallowed. Checks done in callbacks are therefore collected with check_in_callback() and
reported when the tester is closed.
"""

import datetime
import json
import math
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from mavsdk import ComponentType, Configuration, ConnectionResult, Mavsdk  # type: ignore
from mavsdk.plugins.action import Action, ActionError  # type: ignore
from mavsdk.plugins.events import Events  # type: ignore
from mavsdk.plugins.failure import Failure, FailureError, FailureResult  # type: ignore
from mavsdk.plugins.ftp import Ftp, FtpResult  # type: ignore
from mavsdk.plugins.info import Info, InfoError  # type: ignore
from mavsdk.plugins.manual_control import ManualControl  # type: ignore
from mavsdk.plugins.mavlink_direct import MavlinkDirect, MavlinkMessage  # type: ignore
from mavsdk.plugins.mission import Mission, MissionError, MissionItem, MissionPlan  # type: ignore
from mavsdk.plugins.mission_raw import (  # type: ignore
    MissionItem as RawMissionItem, MissionRaw, MissionRawError)
from mavsdk.plugins.offboard import (  # type: ignore
    Attitude, Offboard, OffboardError, PositionNedYaw, VelocityNedYaw)
from mavsdk.plugins.param import Param  # type: ignore
from mavsdk.plugins.telemetry import FlightMode, LandedState, Telemetry, VtolState  # type: ignore

from geometry import CoordinateTransformation, Vec3, point_to_line_distance

# Only print the first few callback check failures, they can come at telemetry rate.
MAX_PRINTED_CALLBACK_FAILURES = 10

# MAVLink constants used in raw mission items and messages.
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_VTOL_LAND = 85
MAV_CMD_DO_JUMP = 177
MAV_CMD_REQUEST_MESSAGE = 512
MAV_FRAME_GLOBAL_INT = 5
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_FRAME_MISSION = 2
MAV_MISSION_TYPE_MISSION = 0
MAV_COMP_ID_AUTOPILOT1 = 1
MAVLINK_MSG_ID_HOME_POSITION = 242
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_IN_PROGRESS = 5

# Like MAVSDK's command sender, but in host time with a longer timeout for sanitizer builds.
COMMAND_ACK_TIMEOUT_S = 1.0
COMMAND_ATTEMPTS = 3

# Frames in which x/y of a raw mission item are latitude/longitude.
MAV_GLOBAL_FRAMES = (0, 3, 5, 6, 10, 11)

MANUAL_CONTROL_RATE_HZ = 50
OFFBOARD_RATE_HZ = 50


TEST_DIR = os.path.dirname(os.path.abspath(__file__))


def plan_file(name: str) -> str:
    """Path of a QGC .plan file next to the tests."""
    return os.path.join(TEST_DIR, name)


def raw_mission_item(seq: int, command: int, frame: int, x: int = 0, y: int = 0, z: float = 0.0,
                     param1: float = 0.0, param2: float = 0.0, current: int = 0,
                     autocontinue: int = 1, mission_type: int = MAV_MISSION_TYPE_MISSION) -> Any:
    return RawMissionItem(
        seq=seq, frame=frame, command=command, current=current, autocontinue=autocontinue,
        param1=param1, param2=param2, param3=0.0, param4=0.0, x=x, y=y, z=z,
        mission_type=mission_type)


def raw_item_local(item: Any, ct: CoordinateTransformation) -> Vec3:
    """Local NED position of a raw mission item in a global frame."""
    north_m, east_m = ct.local_from_global(item.x / 1e7, item.y / 1e7)
    return north_m, east_m, -item.z


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
        self._system: Any = None
        self._action: Any = None
        self._events: Any = None
        self._failure: Any = None
        self._ftp: Any = None
        self._info: Any = None
        self._manual_control: Any = None
        self._mavlink_direct: Any = None
        self._mission: Any = None
        self._mission_raw: Any = None
        self._offboard: Any = None
        self._param: Any = None
        self._telemetry: Any = None

        self._home: Optional[Tuple[float, float]] = None
        self._home_altitude_m: Optional[float] = None
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
        self._system = system

        self._action = Action(system)
        self._events = Events(system)
        self._failure = Failure(system)
        self._ftp = Ftp(system)
        self._info = Info(system)
        self._manual_control = ManualControl(system)
        self._mavlink_direct = MavlinkDirect(system)
        self._mission = Mission(system)
        self._mission_raw = MissionRaw(system)
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
                    # It might have become true while sleeping.
                    return condition()
        else:
            start = time.monotonic()
            while not condition():
                time.sleep(check_interval_s)
                elapsed_s = time.monotonic() - start
                if elapsed_s > timeout_s:
                    log("Timeout, waiting for the vehicle for {:.1f} seconds".format(elapsed_s))
                    return condition()
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

    def get_param_float(self, name: str) -> float:
        return float(self._param.get_param_float(name))

    def get_all_params(self) -> Any:
        """All parameters, downloaded with the PARAM_VALUE stream."""
        return self._param.get_all_params()

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

    def hold(self) -> None:
        self._action.hold()

    # ------------------------------------------------------------------
    # Failure injection
    # ------------------------------------------------------------------

    def inject_failure(self, unit: Any, failure_type: Any, instance: int = 0,
                       expected_result: Any = FailureResult.SUCCESS) -> None:
        """Inject a failure. Needs SYS_FAILURE_EN to be set to succeed."""
        try:
            self._failure.inject(unit, failure_type, instance)
            result = FailureResult.SUCCESS
        except FailureError as e:
            result = e.result
        assert result == expected_result, \
            "Failure injection returned {}, expected {}".format(result, expected_result)

    def enable_actuator_output_status(self) -> None:
        self._telemetry.set_rate_actuator_output_status(20.0)

    def ensure_motors_stopped(self, stopped_mask: int, num_motors: int) -> None:
        """Check that exactly the motors in stopped_mask (0-based bits) are stopped."""
        status = self._telemetry.actuator_output_status()
        assert status.active >= num_motors, \
            "Only {} actuators active, expected {}".format(status.active, num_motors)

        for i in range(num_motors):
            if stopped_mask & (1 << i):
                assert status.actuator[i] <= 901.0, \
                    "Motor {} output {}, expected stopped".format(i, status.actuator[i])
            else:
                assert status.actuator[i] >= 999.0, \
                    "Motor {} output {}, expected running".format(i, status.actuator[i])

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
                self._home_altitude_m = float(gt.absolute_altitude_m)
                return True
            return False

        assert self.poll_condition_with_timeout(got_home, 10), "No ground truth position received"

    def _home_or_fail(self) -> Tuple[float, float]:
        assert self._home is not None, "store_home() was not called"
        return self._home

    def home_absolute_altitude_m(self) -> float:
        assert self._home_altitude_m is not None, "store_home() was not called"
        return self._home_altitude_m

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

    def get_attitude_euler(self) -> Any:
        return self._telemetry.attitude_euler()

    def get_home_position(self, timeout_s: float = 10) -> Dict[str, Any]:
        """Request a fresh HOME_POSITION message and return its fields."""
        received: List[Dict[str, Any]] = []

        def on_message(message: Any, _user_data: Any) -> None:
            if not received:
                received.append(json.loads(message.fields_json))

        handle = self._mavlink_direct.subscribe_message("HOME_POSITION", on_message)
        try:
            # Ask for it rather than relying on the periodic stream timing.
            self.send_command_long(MAV_CMD_REQUEST_MESSAGE, param1=MAVLINK_MSG_ID_HOME_POSITION)
            assert self.poll_condition_with_timeout(lambda: bool(received), timeout_s), \
                "No HOME_POSITION received"
        finally:
            self._mavlink_direct.unsubscribe_message(handle)
        return received[0]

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

    def check_tracks_mission_raw(self, corridor_radius_m: float = 1.0,
                                 reverse: bool = False) -> None:
        """Like check_tracks_mission(), for a mission uploaded with MissionRaw."""
        items = self._mission_raw.download_mission()
        ct = self.get_coordinate_transformation()
        waypoints = {item.seq: raw_item_local(item, ct) for item in items}

        def check(position_velocity_ned: Any, _user_data: Any) -> None:
            progress = self._mission_raw.mission_progress()
            if not 0 < progress.current < progress.total:
                return
            previous = progress.current + 1 if reverse else progress.current - 1
            position = position_velocity_ned.position
            distance_m = point_to_line_distance(
                (position.north_m, position.east_m, position.down_m),
                waypoints.get(previous, (0.0, 0.0, 0.0)),
                waypoints.get(progress.current, (0.0, 0.0, 0.0)))
            self.check_in_callback(
                distance_m < corridor_radius_m,
                "Distance to mission track {:.2f} m exceeds {} m".format(
                    distance_m, corridor_radius_m))

        self._telemetry.set_rate_position_velocity_ned(5)
        self._telemetry.subscribe_position_velocity_ned(check)

    def check_mission_land_within(self, acceptance_radius_m: float) -> None:
        """Check that the vehicle is close to the landing item at the end of the mission."""
        land_item = self._mission_raw.download_mission()[-1]
        assert land_item.command in (MAV_CMD_NAV_LAND, MAV_CMD_NAV_VTOL_LAND), \
            "Last mission item is command {}, not a landing".format(land_item.command)
        distance_m = self._ground_truth_horizontal_distance_to(land_item.x / 1e7,
                                                               land_item.y / 1e7)
        assert distance_m < acceptance_radius_m, \
            "Distance to mission landing {:.2f} m, expected < {} m".format(
                distance_m, acceptance_radius_m)

    def check_mission_item_speed_above(self, item_index: int, min_speed_m_s: float) -> None:
        """Fail the test if the horizontal speed is not above min_speed_m_s while flying to
        mission item item_index."""
        def check(velocity: Any, _user_data: Any) -> None:
            if self._mission.mission_progress().current != item_index:
                return
            speed_m_s = math.hypot(velocity.north_m_s, velocity.east_m_s)
            self.check_in_callback(
                speed_m_s > min_speed_m_s,
                "Speed {:.2f} m/s to mission item {} not above {} m/s".format(
                    speed_m_s, item_index, min_speed_m_s))

        self._telemetry.set_rate_velocity_ned(10)
        self._telemetry.subscribe_velocity_ned(check)

    # ------------------------------------------------------------------
    # Raw missions
    # ------------------------------------------------------------------

    def upload_mission_raw(self, items: List[Any]) -> None:
        self._mission_raw.upload_mission(items)
        # PX4 needs time to realize that it now has a mission available.
        self.sleep_for(1)

    def load_qgc_mission_raw_and_move_here(self, plan_file: str) -> None:
        """Upload a QGC .plan mission, shifted so that it starts at the current position."""
        items = self._mission_raw.import_qgroundcontrol_mission(plan_file).mission_items
        self._move_mission_raw_here(items)
        self.upload_mission_raw(items)

    def _move_mission_raw_here(self, items: List[Any]) -> None:
        position = self._telemetry.position()
        assert math.isfinite(position.latitude_deg) and math.isfinite(position.longitude_deg), \
            "No global position"

        offset_x = items[0].x - int(1e7 * position.latitude_deg)
        offset_y = items[0].y - int(1e7 * position.longitude_deg)
        for item in items:
            if item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT:
                item.x -= offset_x
            item.y -= offset_y

    def prepare_multicopter_mission_with_do_jump(self, options: MissionOptions,
                                                 jump_repeats: int) -> int:
        """Upload a mission (relative to home) with a DO_JUMP looping back to an earlier waypoint.
        The mission lands at the end so it terminates on its own. Return the index of the
        DO_JUMP item."""
        ct = self.get_coordinate_transformation()
        altitude_m = options.relative_altitude_m
        leg_m = options.leg_length_m

        def position_item(seq: int, command: int, north_m: float, east_m: float, z: float,
                          current: int = 0) -> Any:
            latitude_deg, longitude_deg = ct.global_from_local(north_m, east_m)
            return raw_mission_item(seq, command, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    x=round(latitude_deg * 1e7), y=round(longitude_deg * 1e7),
                                    z=z, current=current)

        jump_index = 3
        self.upload_mission_raw([
            position_item(0, MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, altitude_m, current=1),
            position_item(1, MAV_CMD_NAV_WAYPOINT, leg_m, 0.0, altitude_m),
            position_item(2, MAV_CMD_NAV_WAYPOINT, leg_m, leg_m, altitude_m),
            raw_mission_item(jump_index, MAV_CMD_DO_JUMP, MAV_FRAME_MISSION,
                             param1=1.0, param2=float(jump_repeats)),
            position_item(4, MAV_CMD_NAV_WAYPOINT, 0.0, leg_m, altitude_m),
            position_item(5, MAV_CMD_NAV_LAND, 0.0, 0.0, 0.0),
        ])
        return jump_index

    def start_mission_raw_and_wait_for_sequence(self, sequence_number: int,
                                                timeout_s: float = 180) -> None:
        self._mission_raw.start_mission()
        assert self.poll_condition_with_timeout(
            lambda: self._mission_raw.mission_progress().current >= sequence_number, timeout_s), \
            "Mission item {} not reached within {} s".format(sequence_number, timeout_s)

    def execute_rtl_when_reaching_mission_sequence(self, sequence_number: int,
                                                   timeout_s: float = 180) -> None:
        self.start_mission_raw_and_wait_for_sequence(sequence_number, timeout_s)
        self.execute_rtl()

    def execute_mission_raw(self) -> None:
        self._mission_raw.start_mission()

        def finished() -> bool:
            try:
                return bool(self._mission_raw.is_mission_finished())
            except MissionRawError:
                return False

        assert self.poll_condition_with_timeout(finished, 300), \
            "Mission not finished within 300 s"

    def send_set_current_mission_item(self, index: int) -> None:
        """Send a single MISSION_SET_CURRENT, like a one-shot operator/GCS action.

        MissionRaw.set_current_mission_item() is deliberately not used: it re-sends until
        MISSION_CURRENT echoes the requested seq. For a DO_JUMP item the navigator reports the
        jump target instead, so MAVSDK would keep re-sending and resetting the mission.
        """
        self.send_mavlink_message("MISSION_SET_CURRENT", {
            "target_system": self._target_system_id(),
            "target_component": MAV_COMP_ID_AUTOPILOT1,
            "seq": index,
        })

    # ------------------------------------------------------------------
    # MAVLink
    # ------------------------------------------------------------------

    def _target_system_id(self) -> int:
        return int(self._system.get_system_id())

    def send_mavlink_message(self, name: str, fields: Dict[str, Any]) -> None:
        # NaN is not valid JSON; MAVSDK takes null as NaN for float fields.
        def nan_to_null(value: Any) -> Any:
            if isinstance(value, float) and math.isnan(value):
                return None
            if isinstance(value, list):
                return [nan_to_null(v) for v in value]
            return value

        fields_json = json.dumps({k: nan_to_null(v) for k, v in fields.items()}, allow_nan=False)
        # The sender IDs are filled in by MAVSDK.
        self._mavlink_direct.send_message(MavlinkMessage(
            message_name=name, system_id=0, component_id=0, target_system_id=0,
            target_component_id=0, fields_json=fields_json))

    def send_command_long(self, command: int, param1: float = 0.0, param2: float = 0.0,
                          param3: float = 0.0, param4: float = 0.0, param5: float = 0.0,
                          param6: float = 0.0, param7: float = 0.0,
                          expected_result: int = MAV_RESULT_ACCEPTED) -> None:
        """Send a COMMAND_LONG and check its COMMAND_ACK."""
        self._send_command("COMMAND_LONG", {
            "target_system": self._target_system_id(),
            "target_component": MAV_COMP_ID_AUTOPILOT1,
            "command": command, "confirmation": 0,
            "param1": param1, "param2": param2, "param3": param3, "param4": param4,
            "param5": param5, "param6": param6, "param7": param7,
        }, expected_result)

    def send_command_int(self, command: int, param1: float = 0.0, param2: float = 0.0,
                         param3: float = 0.0, param4: float = 0.0, x: int = 0, y: int = 0,
                         z: float = 0.0, frame: int = MAV_FRAME_GLOBAL_INT,
                         expected_result: int = MAV_RESULT_ACCEPTED) -> None:
        """Send a COMMAND_INT and check its COMMAND_ACK."""
        self._send_command("COMMAND_INT", {
            "target_system": self._target_system_id(),
            "target_component": MAV_COMP_ID_AUTOPILOT1,
            "frame": frame, "command": command, "current": 0, "autocontinue": 0,
            "param1": param1, "param2": param2, "param3": param3, "param4": param4,
            "x": x, "y": y, "z": z,
        }, expected_result)

    def _send_command(self, message_name: str, fields: Dict[str, Any],
                      expected_result: int) -> None:
        """Send a command until it is acked, then check the result."""
        command = fields["command"]
        results: List[int] = []
        acked = threading.Event()

        def on_ack(message: Any, _user_data: Any) -> None:
            ack = json.loads(message.fields_json)
            if ack["command"] == command and ack["result"] != MAV_RESULT_IN_PROGRESS:
                results.append(int(ack["result"]))
                acked.set()

        handle = self._mavlink_direct.subscribe_message("COMMAND_ACK", on_ack)
        try:
            for attempt in range(COMMAND_ATTEMPTS):
                if message_name == "COMMAND_LONG":
                    fields["confirmation"] = attempt
                self.send_mavlink_message(message_name, fields)
                if acked.wait(COMMAND_ACK_TIMEOUT_S):
                    break
        finally:
            self._mavlink_direct.unsubscribe_message(handle)

        assert results, "No COMMAND_ACK for command {} after {} attempts".format(
            command, COMMAND_ATTEMPTS)
        assert results[0] == expected_result, \
            "Command {} result {}, expected {}".format(command, results[0], expected_result)

    # ------------------------------------------------------------------
    # Position monitors
    # ------------------------------------------------------------------

    def wait_for_position_ned(self, condition: Callable[[Any], bool], timeout_s: float,
                              rate_hz: float = 10.0) -> None:
        """Wait until condition(position_velocity_ned) is true for a telemetry sample. Unlike
        polling, this sees every sample, so a short crossing cannot be missed."""
        reached = threading.Event()

        def on_sample(sample: Any, _user_data: Any) -> None:
            if condition(sample):
                reached.set()

        self._telemetry.set_rate_position_velocity_ned(rate_hz)
        handle = self._telemetry.subscribe_position_velocity_ned(on_sample)
        try:
            assert self.poll_condition_with_timeout(reached.is_set, timeout_s), \
                "Position condition not met within {} s".format(timeout_s)
        finally:
            self._telemetry.unsubscribe_position_velocity_ned(handle)

    def check_position_ned_for(self, check: Callable[[Any], Optional[str]], duration_s: float,
                               rate_hz: float = 10.0) -> None:
        """Run check(position_velocity_ned) on every sample for duration_s. It returns an error
        message when the check fails, which then fails the test."""
        def on_sample(sample: Any, _user_data: Any) -> None:
            error = check(sample)
            self.check_in_callback(error is None, error or "")

        self._telemetry.set_rate_position_velocity_ned(rate_hz)
        handle = self._telemetry.subscribe_position_velocity_ned(on_sample)
        try:
            self.sleep_for(duration_s)
        finally:
            self._telemetry.unsubscribe_position_velocity_ned(handle)
        self._assert_no_callback_failures()

    def _assert_no_callback_failures(self) -> None:
        with self._failures_lock:
            failures = list(self._callback_failures)
        assert not failures, "{} checks failed, first: {}".format(len(failures), failures[0])

    # ------------------------------------------------------------------
    # Fixed-wing metrics
    # ------------------------------------------------------------------

    def enable_fixedwing_metrics(self) -> None:
        self._telemetry.set_rate_fixedwing_metrics(10.0)

    def check_airspeed_is_valid(self) -> None:
        # Once the airspeed is invalidated, it is sent as NaN and stays so with the default
        # parameters.
        airspeed = self._telemetry.fixedwing_metrics().airspeed_m_s
        assert math.isfinite(airspeed), "Airspeed is invalid"

    def check_airspeed_is_invalid(self) -> None:
        airspeed = self._telemetry.fixedwing_metrics().airspeed_m_s
        assert not math.isfinite(airspeed), "Airspeed {} m/s is still valid".format(airspeed)

    # ------------------------------------------------------------------
    # FTP
    # ------------------------------------------------------------------

    def _wait_for_ftp(self, start: Callable[[Callable[[Any, Any, Any], None]], None],
                      timeout_s: float) -> None:
        results: List[Any] = []
        done = threading.Event()

        def on_progress(result: Any, _progress: Any, _user_data: Any) -> None:
            if result != FtpResult.NEXT and not done.is_set():
                results.append(result)
                done.set()

        start(on_progress)
        # Host time: the transfer is not tied to the simulation.
        assert done.wait(timeout_s), "FTP transfer did not finish within {} s".format(timeout_s)
        assert results[0] == FtpResult.SUCCESS, "FTP transfer failed: {}".format(results[0])

    def ftp_download(self, remote_path: str, local_dir: str, use_burst: bool = True,
                     timeout_s: float = 60) -> None:
        self._wait_for_ftp(
            lambda cb: self._ftp.download_async(remote_path, local_dir, use_burst, cb), timeout_s)

    def ftp_upload(self, local_path: str, remote_dir: str, timeout_s: float = 60) -> None:
        self._wait_for_ftp(
            lambda cb: self._ftp.upload_async(local_path, remote_dir, cb), timeout_s)

    # ------------------------------------------------------------------
    # Manual control
    # ------------------------------------------------------------------

    def _send_manual_control(self, duration_s: float, x: float, y: float, z: float, r: float,
                             stop_when_landed: bool = False) -> None:
        for _ in range(int(duration_s * MANUAL_CONTROL_RATE_HZ)):
            self._manual_control.set_manual_control_input(x, y, z, r)
            self.sleep_for(1.0 / MANUAL_CONTROL_RATE_HZ)
            if stop_when_landed and not self._telemetry.in_air():
                return

    def _fly_forward_manually(self, start_mode: Callable[[], None]) -> None:
        # Keep sending centered sticks in between so RC stays available.
        self._send_manual_control(1, 0.0, 0.0, 0.5, 0.0)
        start_mode()
        self._send_manual_control(1, 0.0, 0.0, 0.5, 0.0)
        self.store_home()
        self._send_manual_control(1, 0.0, 0.0, 0.5, 0.0)
        self.wait_until_ready()
        self._send_manual_control(1, 0.0, 0.0, 0.5, 0.0)
        self.arm()
        self._send_manual_control(1, 0.0, 0.0, 0.5, 0.0)

        log("Climbing")
        self._send_manual_control(5, 0.0, 0.0, 1.0, 0.0)
        log("Flying forward")
        self._send_manual_control(10, 0.5, 0.0, 0.5, 0.0)
        log("Descending until landed")
        self._send_manual_control(60, 0.0, 0.0, 0.0, 0.0, stop_when_landed=True)

    def fly_forward_in_posctl(self) -> None:
        self._fly_forward_manually(self._manual_control.start_position_control)

    def fly_forward_in_altctl(self) -> None:
        self._fly_forward_manually(self._manual_control.start_altitude_control)

    # ------------------------------------------------------------------
    # Offboard
    # ------------------------------------------------------------------

    def _send_attitude(self, duration_s: float, attitude: Any) -> None:
        for _ in range(int(duration_s * OFFBOARD_RATE_HZ)):
            self._offboard.set_attitude(attitude)
            self.sleep_for(1.0 / OFFBOARD_RATE_HZ)

    def fly_forward_in_offboard_attitude(self) -> None:
        # This does not depend on a valid position estimate. Wait for raw GPS and a stable
        # attitude estimate.
        def ready() -> bool:
            attitude = self._telemetry.attitude_euler()
            return bool(self._telemetry.raw_gps().altitude_ellipsoid_m > 0.0
                        and abs(attitude.roll_deg) < 5.0 and abs(attitude.pitch_deg) < 5.0)

        assert self.poll_condition_with_timeout(ready, 20), "No raw GPS or attitude not stable"
        start_altitude_m = float(self._telemetry.raw_gps().altitude_ellipsoid_m)

        level = Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, thrust_value=0.0)
        self._offboard.set_attitude(level)
        self._start_offboard_with_retry(lambda: self._offboard.set_attitude(level))

        assert self.poll_condition_with_timeout(
            lambda: bool(self._telemetry.health().is_armable), 20), "System not armable"
        self.arm()

        log("Climbing")
        climb = Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, thrust_value=0.8)
        while self._telemetry.raw_gps().altitude_ellipsoid_m - start_altitude_m < 10.0:
            self._offboard.set_attitude(climb)
            self.sleep_for(1.0 / OFFBOARD_RATE_HZ)

        log("Flying forward")
        forward = Attitude(roll_deg=0.0, pitch_deg=-20.0, yaw_deg=0.0, thrust_value=0.8)
        self._send_attitude(3, forward)

        estimate = self._telemetry.attitude_euler()
        assert abs(forward.roll_deg - estimate.roll_deg) < 5.0, \
            "Roll {:.1f} deg, expected {}".format(estimate.roll_deg, forward.roll_deg)
        assert abs(forward.pitch_deg - estimate.pitch_deg) < 5.0, \
            "Pitch {:.1f} deg, expected {}".format(estimate.pitch_deg, forward.pitch_deg)

        log("Descending")
        self._send_attitude(6, Attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0,
                                        thrust_value=0.4))
        self._offboard.set_attitude(level)

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
