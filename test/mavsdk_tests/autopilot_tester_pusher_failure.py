"""AutopilotTester for the pusher failure detection of the SIH standard VTOL with pusher assist.

The pusher is stopped with an undetected motor failure (`failure motor wrong`): its output is cut,
but it stays in the control allocation and no motor failure is reported, as with a pusher without
ESC telemetry. The checks only use what the vehicle reports (quad-chute events, VTOL state,
outputs), not how a failure is detected, so any detector can be measured with the same flights.
"""

import threading
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

from mavsdk.plugins.action import ActionError  # type: ignore
from mavsdk.plugins.failure import FailureType, FailureUnit  # type: ignore
from mavsdk.plugins.telemetry import VtolState  # type: ignore

from autopilot_tester import (MAV_CMD_NAV_LAND, MAV_CMD_NAV_WAYPOINT,
                              MAV_FRAME_GLOBAL_RELATIVE_ALT, AutopilotTester, log,
                              raw_mission_item)

# The pusher is motor 5 (1-based failure instance) on output channel 8.
PUSHER_MOTOR_INSTANCE = 5
PUSHER_OUTPUT_INDEX = 7
# Outputs are 1000 + 1000 * throttle; a stopped or unused pusher reads 900.
PUSHER_PUSHING_OUTPUT = 1100.0
PUSHER_STOPPED_OUTPUT = 901.0

QUADCHUTE_EVENT_PREFIX = "vtol_att_ctrl_quadchute"
PUSHER_QUADCHUTE_EVENT = "vtol_att_ctrl_quadchute_pusher"

# Sample the VTOL state fast enough not to miss a short fixed-wing phase.
VTOL_STATE_RATE_HZ = 10.0


class FixedWingReached(AssertionError):
    pass


class QuadchuteMissing(AssertionError):
    pass


@dataclass
class Wind:
    """Constant SIH wind. SIH_WIND_N and SIH_WIND_E are the speeds of the wind blowing from north
    and from east, so a positive from_north_m_s is a headwind when flying north."""
    from_north_m_s: float = 0.0
    from_east_m_s: float = 0.0


@dataclass
class Quadchute:
    time_s: float
    event_name: str


class AutopilotTesterPusherFailure(AutopilotTester):

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        super().__init__(speed_factor)
        self._record_lock = threading.Lock()
        self._quadchutes: List[Quadchute] = []
        self._vtol_states: List[Tuple[float, Any]] = []
        self._pusher_failure_time_s: Optional[float] = None

    def connect(self, url: str) -> None:
        super().connect(url)
        self._events.subscribe_events(self._on_event)
        self._telemetry.subscribe_vtol_state(self._on_vtol_state)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def take_off_with_pusher_assist(self, altitude_m: float, use_airspeed: bool = True) -> None:
        """Take off with pusher assist and failure injection enabled, then hold."""
        self.wait_until_ready()
        self.set_param_int("VT_FWD_THRUST_EN", 1)
        self.set_param_int("SYS_FAILURE_EN", 1)
        # The control allocation only applies injected motor stops with a failure mode set.
        self.set_param_int("CA_FAILURE_MODE", 1)
        # Airspeed-less setup as in vtol_without_airspeed_sensor.md; the sensor stays installed.
        self.set_param_int("FW_USE_AIRSPD", 1 if use_airspeed else 0)
        self.enable_actuator_output_status()
        self._telemetry.set_rate_vtol_state(VTOL_STATE_RATE_HZ)

        self.set_takeoff_altitude(altitude_m)
        # Necessary for the takeoff altitude to be applied properly.
        self.sleep_for(1)
        self.arm()
        self.takeoff()
        self.wait_until_altitude(altitude_m, 60)

    def set_wind(self, wind: Wind, settle_s: float) -> None:
        # Wind on the parked SIH vehicle can fail its preflight checks, so set it in the air.
        log("Wind from north {} m/s, from east {} m/s".format(
            wind.from_north_m_s, wind.from_east_m_s))
        self.set_param_float("SIH_WIND_N", wind.from_north_m_s)
        self.set_param_float("SIH_WIND_E", wind.from_east_m_s)
        self.sleep_for(settle_s)

    def keep_heading(self) -> None:
        """Turn weathervane off, so the nose stays north and the wind a head- or tailwind."""
        self.set_param_int("WV_EN", 0)

    def enable_fixed_wing_propulsion_check(self, esc_current_max_a: float,
                                           propulsion_current_min_a: float) -> None:
        """Simulate the ESC and battery current (SIH_ESC_I_MAX) and enable the fixed-wing
        propulsion failure check (COM_EF_CURR)."""
        self.set_param_float("SIH_ESC_I_MAX", esc_current_max_a)
        self.set_param_float("COM_EF_CURR", propulsion_current_min_a)

    def start_mission(self, waypoints: List[Tuple[float, float]], altitude_m: float) -> None:
        """Fly to each (north, east) waypoint relative to home, in the current VTOL mode, and land
        at the last one."""
        ct = self.get_coordinate_transformation()

        def item(seq: int, command: int, north_m: float, east_m: float, z: float) -> Any:
            latitude_deg, longitude_deg = ct.global_from_local(north_m, east_m)
            return raw_mission_item(seq, command, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    x=round(latitude_deg * 1e7), y=round(longitude_deg * 1e7),
                                    z=z, current=1 if seq == 0 else 0)

        items = [item(seq, MAV_CMD_NAV_WAYPOINT, north_m, east_m, altitude_m)
                 for seq, (north_m, east_m) in enumerate(waypoints)]
        items.append(item(len(items), MAV_CMD_NAV_LAND, *waypoints[-1], 0.0))
        self.upload_mission_raw(items)
        self._mission_raw.start_mission()

    def wait_until_north_of(self, north_m: float, timeout_s: float) -> None:
        self.wait_for_position_ned(lambda s: bool(s.position.north_m > north_m), timeout_s)

    # ------------------------------------------------------------------
    # Pusher
    # ------------------------------------------------------------------

    def _pusher_output(self) -> float:
        return float(self._telemetry.actuator_output_status().actuator[PUSHER_OUTPUT_INDEX])

    def check_pusher_pushing(self) -> None:
        """Check that pusher assist is pushing, otherwise the flight tests nothing."""
        output = self._pusher_output()
        assert output >= PUSHER_PUSHING_OUTPUT, \
            "Pusher output {:.0f}, expected pusher assist to push".format(output)

    def fail_pusher(self) -> float:
        """Stop the pusher, return the time of the failure."""
        self.inject_failure(FailureUnit.SYSTEM_MOTOR, FailureType.WRONG, PUSHER_MOTOR_INSTANCE)
        self._pusher_failure_time_s = self.time_s()
        log("Pusher stopped")
        return self._pusher_failure_time_s

    def pusher_failure_time_s(self) -> float:
        assert self._pusher_failure_time_s is not None, "fail_pusher() was not called"
        return self._pusher_failure_time_s

    def check_pusher_output_above(self, output: float, duration_s: float) -> None:
        """Check that the pusher output stays at least this high for this time, after it got
        there."""
        assert self.poll_condition_with_timeout(lambda: self._pusher_output() >= output, 10), \
            "Pusher output {:.0f}, expected at least {:.0f}".format(self._pusher_output(), output)
        end_s = self.time_s() + duration_s
        while self.time_s() < end_s:
            current_output = self._pusher_output()
            assert current_output >= output, \
                "Pusher output {:.0f}, expected at least {:.0f}".format(current_output, output)
            self.sleep_for(0.1)

    def check_pusher_stopped(self) -> None:
        """Check that the injected failure cut the pusher output while it should push."""
        assert self.poll_condition_with_timeout(
            lambda: self._pusher_output() <= PUSHER_STOPPED_OUTPUT, 1), \
            "Pusher output {:.0f}, expected the pusher to be stopped".format(self._pusher_output())

    # ------------------------------------------------------------------
    # Quad-chute and VTOL state
    # ------------------------------------------------------------------

    def wait_for_quadchute(self, since_s: float, timeout_s: float,
                           expected_event: Optional[str] = None) -> float:
        """Wait for the first quad-chute after since_s, return its delay."""
        received = self.poll_condition_with_timeout(
            lambda: bool(self._quadchutes_since(since_s)), timeout_s)
        if not received:
            raise QuadchuteMissing("No quad-chute within {} s".format(timeout_s))

        quadchute = self._quadchutes_since(since_s)[0]
        if expected_event is not None:
            assert quadchute.event_name == expected_event, \
                "Quad-chute {}, expected {}".format(quadchute.event_name, expected_event)

        delay_s = quadchute.time_s - since_s
        log("Quad-chute {} after {:.1f} s".format(quadchute.event_name, delay_s))
        if delay_s > timeout_s:
            raise QuadchuteMissing("Quad-chute after {:.1f} s, expected within {} s".format(
                delay_s, timeout_s))
        return delay_s

    def wait_for_pusher_quadchute(self, timeout_s: float) -> float:
        """Wait for the pusher failure quad-chute after fail_pusher(), return the detection
        time."""
        assert self._pusher_failure_time_s is not None, "fail_pusher() was not called"
        return self.wait_for_quadchute(
            self._pusher_failure_time_s, timeout_s, PUSHER_QUADCHUTE_EVENT)

    def start_front_transition(self) -> float:
        """Command a front transition, return the time it was commanded once it started."""
        transition_time_s = self.time_s()
        self.transition_to_fixedwing()
        self.wait_until_transition_started(transition_time_s, 5)
        return transition_time_s

    def try_transition_to_fixedwing(self) -> None:
        """Command a front transition once; it may be denied."""
        try:
            self.transition_to_fixedwing()
        except ActionError as e:
            log("Front transition denied: {}".format(e))

    def check_no_quadchute(self) -> None:
        quadchutes = self._quadchutes_since(0.0)
        assert not quadchutes, "Quad-chute {} at {:.1f} s".format(
            quadchutes[0].event_name, quadchutes[0].time_s)

    def wait_until_transition_started(self, since_s: float, timeout_s: float) -> None:
        assert self.poll_condition_with_timeout(
            lambda: VtolState.TRANSITION_TO_FW in self._vtol_states_since(since_s), timeout_s), \
            "No front transition started within {} s".format(timeout_s)

    def check_never_fixed_wing_since(self, since_s: float, duration_s: float) -> None:
        """Fail if fixed-wing mode is reached between since_s and duration_s from now."""
        self.sleep_for(duration_s)
        if VtolState.FW in self._vtol_states_since(since_s):
            raise FixedWingReached("Fixed-wing mode reached")

    def check_only_multicopter_since(self, since_s: float) -> None:
        states = self._vtol_states_since(since_s)
        assert states, "No VTOL state received"
        other = [state for state in states if state != VtolState.MC]
        assert not other, "VTOL state {}, expected multicopter only".format(other[0])

    def time_s(self) -> float:
        """Autopilot time."""
        return self._autopilot_time_us() / 1e6

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _quadchutes_since(self, since_s: float) -> List[Quadchute]:
        with self._record_lock:
            return [q for q in self._quadchutes if q.time_s >= since_s]

    def _vtol_states_since(self, since_s: float) -> List[Any]:
        with self._record_lock:
            return [state for time_s, state in self._vtol_states if time_s >= since_s]

    def _on_event(self, event: Any, _user_data: Any) -> None:
        if event.event_name.startswith(QUADCHUTE_EVENT_PREFIX):
            with self._record_lock:
                self._quadchutes.append(Quadchute(self.time_s(), str(event.event_name)))

    def _on_vtol_state(self, state: Any, _user_data: Any) -> None:
        with self._record_lock:
            self._vtol_states.append((self.time_s(), state))
