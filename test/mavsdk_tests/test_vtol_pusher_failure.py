"""Pusher failure detection of standard VTOLs (VT_EF_TIME, VT_EF_FT_ACC, COM_EF_CURR).

What the detection must achieve, whatever the detector:
1. A pusher that failed before or early in a front transition triggers the pusher failure
   quad-chute within FT_DETECTION_TIME_S of the transition start, with or without airspeed
   sensor. The vehicle never reaches fixed-wing flight and lands in multicopter mode.
2. A pusher that fails while the vehicle flies forward with pusher assist (VT_FWD_THRUST_EN) in
   multicopter mode triggers the pusher failure quad-chute within DETECTION_TIME_S. The vehicle
   then stays in multicopter mode and lands.
3. A working pusher triggers no quad-chute, in multicopter flight, in front transitions and in
   fixed-wing flight.
4. With a battery current measurement, a pusher that fails in fixed-wing flight triggers the pusher
   failure quad-chute within FW_DETECTION_TIME_S, with or without airspeed sensor. The vehicle
   then flies in multicopter mode and lands.

Accepted, not required: a headwind stronger than the pusher may be reported as a pusher failure,
since the vehicle cannot make progress with pusher assist anyway.
Known gaps of the VT_EF_* detector: a tailwind that carries the vehicle within VT_EF_VEL_ERR of
its velocity setpoint hides the failure, a pusher that fails in hover is only detected on the
next pusher-assisted leg or at the start of the next front transition, and a pusher that fails
after the transition check passed (about 3.4 s into the transition) is only detected once in
fixed-wing flight, with COM_EF_CURR set and a battery current measurement.

The detection times are logged, so detectors can be compared with the same flights:
    test/mavsdk_tests/mavsdk_test_runner.py --speed-factor 10 --model standard_vtol \\
        --case '*pusher*' test/mavsdk_tests/configs/sih-sitl.json
SIH has constant wind only and a 0.2 kg vehicle, so the flights check behavior, not tuning.
"""

import pytest

from autopilot_tester_pusher_failure import (PUSHER_QUADCHUTE_EVENT,
                                             AutopilotTesterPusherFailure, Wind)

ALTITUDE_M = 20.0
SETTLE_S = 10.0
DETECTION_TIME_S = 10.0
# A failed pusher has to stop the front transition before the open-loop transition would complete
# without airspeed (VT_F_TR_OL_TM default).
FT_DETECTION_TIME_S = 6.0
# Fixed-wing flight: COM_EF_TIME (5 s default) once the throttle is above COM_EF_THROT (0.5
# default), which SIH cruises above (about 0.6).
FW_DETECTION_TIME_S = 10.0
# High enough to glide while the failure is detected.
FW_ALTITUDE_M = 60.0
# The SIH motors draw 20 A at full output (SIM_BAT_I_MOT), growing with output^1.5. In fixed-wing
# flight the vehicle then draws 0.5 A (SIM_BAT_I_IDLE) with a dead pusher, and 7.6 A at 0.5 throttle
# with a working one: 3 A splits them.
MOTOR_CURRENT_MAX_A = 20.0
PROPULSION_CURRENT_MIN_A = 3.0
# Pusher output at the COM_EF_THROT default.
GATE_OUTPUT = 1500.0
# Front transition backstop, the VT_TRANS_TIMEOUT default. PX4 scales it with air density
# (about 16 s at the CI home altitude), which the 5 s margin below covers.
TRANSITION_TIMEOUT_S = 15.0

# SIH pusher assist makes progress against 7 m/s of headwind and drifts back at 8 m/s.
NO_WIND = Wind()
HEADWIND = Wind(from_north_m_s=3.0)
CROSSWIND = Wind(from_east_m_s=3.0)
# Weak enough for pusher assist to still push before the failure, which it does not at 3 m/s.
TAILWIND = Wind(from_north_m_s=-1.0)
STRONG_WIND = Wind(from_north_m_s=5.0)
FT_TAILWIND = Wind(from_north_m_s=-3.0)


@pytest.mark.vtol_pusher_failure
@pytest.mark.parametrize("wind", [NO_WIND, HEADWIND, CROSSWIND, TAILWIND],
                         ids=["no_wind", "headwind", "crosswind", "tailwind"])
def test_pusher_failure_in_multicopter_cruise(
        tester_pusher_failure: AutopilotTesterPusherFailure, wind: Wind) -> None:
    """Requirement 2: the pusher stops on a mission leg flown north."""
    tester = tester_pusher_failure
    tester.take_off_with_pusher_assist(ALTITUDE_M)
    tester.set_wind(wind, SETTLE_S)
    tester.start_mission([(300.0, 0.0)], ALTITUDE_M)
    tester.wait_until_north_of(40.0, 60)
    tester.check_pusher_pushing()

    failure_time_s = tester.fail_pusher()
    tester.check_pusher_stopped()
    tester.wait_for_pusher_quadchute(DETECTION_TIME_S)

    # Right away, during the failsafe hold (COM_FAIL_ACT_T): Return rejects front transitions
    # outright, which would test nothing.
    tester.try_transition_to_fixedwing()
    tester.wait_until_disarmed(240)
    tester.check_only_multicopter_since(failure_time_s)


@pytest.mark.vtol_pusher_failure
@pytest.mark.parametrize("wind", [NO_WIND, STRONG_WIND], ids=["no_wind", "strong_wind"])
def test_working_pusher_in_multicopter_mission(
        tester_pusher_failure: AutopilotTesterPusherFailure, wind: Wind) -> None:
    """Requirement 3: legs into, across and with the wind (strong_wind case). The waypoints
    keep the nose north, as weathervane (WV_EN, on for VTOLs) would in this wind, so the
    vehicle flies the downwind leg tail first and the pusher idles."""
    tester = tester_pusher_failure
    tester.request_ground_truth()
    tester.take_off_with_pusher_assist(ALTITUDE_M)
    tester.set_wind(wind, SETTLE_S)
    tester.start_mission([(100.0, 0.0), (100.0, 60.0), (0.0, 60.0)], ALTITUDE_M)
    tester.wait_until_north_of(40.0, 60)
    tester.check_pusher_pushing()

    tester.wait_until_disarmed(450)
    tester.check_no_quadchute()
    tester.check_only_multicopter_since(0.0)
    # A failsafe would have landed elsewhere.
    tester.check_mission_land_within(5.0)


@pytest.mark.vtol_pusher_failure
def test_working_pusher_holding_position_in_strong_wind(
        tester_pusher_failure: AutopilotTesterPusherFailure) -> None:
    """Requirement 3: the pusher pushes against the wind, but the vehicle is not asked to
    move."""
    tester = tester_pusher_failure
    tester.take_off_with_pusher_assist(ALTITUDE_M)
    tester.set_wind(STRONG_WIND, SETTLE_S)
    tester.check_pusher_pushing()

    tester.sleep_for(60)
    tester.check_pusher_pushing()
    tester.wait_until_speed_lower_than(0.5, 5)
    tester.check_no_quadchute()


@pytest.mark.vtol_pusher_failure
@pytest.mark.parametrize("use_airspeed", [True, False], ids=["airspeed", "no_airspeed"])
def test_front_transition_with_failed_pusher(
        tester_pusher_failure: AutopilotTesterPusherFailure, use_airspeed: bool) -> None:
    """Requirement 1: the pusher stops in hover, then a front transition is commanded."""
    tester = tester_pusher_failure
    tester.take_off_with_pusher_assist(ALTITUDE_M, use_airspeed)
    tester.fail_pusher()
    tester.sleep_for(2)

    transition_time_s = tester.start_front_transition()
    tester.sleep_for(1)
    tester.check_pusher_stopped()
    check_front_transition_stopped(tester, transition_time_s)


@pytest.mark.vtol_pusher_failure
@pytest.mark.parametrize("use_airspeed", [True, False], ids=["airspeed", "no_airspeed"])
def test_pusher_failure_early_in_front_transition(
        tester_pusher_failure: AutopilotTesterPusherFailure, use_airspeed: bool) -> None:
    """Requirement 1: the pusher stops 0.5 s into a front transition from hover, while it ramps up
    to the transition throttle (VT_PSHER_SLEW). Later, the SIH vehicle could already be fast
    enough to complete the transition with airspeed."""
    tester = tester_pusher_failure
    tester.take_off_with_pusher_assist(ALTITUDE_M, use_airspeed)

    transition_time_s = tester.start_front_transition()
    tester.sleep_for(0.5)
    tester.fail_pusher()
    tester.check_pusher_stopped()
    check_front_transition_stopped(tester, transition_time_s)


@pytest.mark.vtol_pusher_failure
def test_front_transition_after_pusher_failure_in_cruise(
        tester_pusher_failure: AutopilotTesterPusherFailure) -> None:
    """Requirement 1: the pusher stops on a pusher-assisted mission leg, and a front transition
    is commanded 1 s later, before the multicopter check reports it, while the vehicle still moves
    forward."""
    tester = tester_pusher_failure
    tester.take_off_with_pusher_assist(ALTITUDE_M)
    tester.start_mission([(300.0, 0.0)], ALTITUDE_M)
    tester.wait_until_north_of(40.0, 60)
    tester.check_pusher_pushing()
    tester.fail_pusher()
    tester.check_pusher_stopped()
    tester.sleep_for(1)

    transition_time_s = tester.start_front_transition()
    check_front_transition_stopped(tester, transition_time_s)


@pytest.mark.vtol_pusher_failure
@pytest.mark.parametrize("use_airspeed,wind", [
    (True, NO_WIND), (False, NO_WIND), (False, STRONG_WIND), (False, FT_TAILWIND),
], ids=["airspeed", "no_airspeed", "no_airspeed_headwind", "no_airspeed_tailwind"])
def test_front_transition_with_working_pusher(
        tester_pusher_failure: AutopilotTesterPusherFailure, use_airspeed: bool,
        wind: Wind) -> None:
    """Requirement 3: front transitions from hover with the nose north, so the wind is a head- or
    tailwind."""
    tester = tester_pusher_failure
    tester.take_off_with_pusher_assist(ALTITUDE_M, use_airspeed)
    tester.keep_heading()
    tester.set_wind(wind, SETTLE_S)

    tester.start_front_transition()
    tester.wait_until_fixedwing(TRANSITION_TIMEOUT_S + 5)
    tester.sleep_for(5)
    tester.check_no_quadchute()


@pytest.mark.vtol_pusher_failure
def test_pusher_failure_late_in_front_transition(
        tester_pusher_failure: AutopilotTesterPusherFailure) -> None:
    """Requirement 4: without airspeed, the pusher stops after the transition check passed. Today
    the open-loop transition completes, and the fixed-wing check reports the failure."""
    tester = tester_pusher_failure
    tester.enable_fixed_wing_propulsion_check(MOTOR_CURRENT_MAX_A, PROPULSION_CURRENT_MIN_A)
    tester.take_off_with_pusher_assist(FW_ALTITUDE_M, use_airspeed=False)

    tester.start_front_transition()
    # The transition check decides 3.4 s into the transition, at the default VT_PSHER_SLEW.
    tester.sleep_for(5)
    tester.fail_pusher()
    tester.check_pusher_stopped()
    check_fixed_wing_flight_stopped(tester)


@pytest.mark.vtol_pusher_failure
@pytest.mark.parametrize("wind", [NO_WIND, STRONG_WIND], ids=["no_wind", "strong_headwind"])
def test_pusher_failure_in_fixed_wing_flight(
        tester_pusher_failure: AutopilotTesterPusherFailure, wind: Wind) -> None:
    """Requirement 4: the pusher stops on a fixed-wing mission leg flown north."""
    tester = tester_pusher_failure
    tester.enable_fixed_wing_propulsion_check(MOTOR_CURRENT_MAX_A, PROPULSION_CURRENT_MIN_A)
    tester.take_off_with_pusher_assist(FW_ALTITUDE_M)
    tester.set_wind(wind, SETTLE_S)
    tester.start_front_transition()
    tester.wait_until_fixedwing(TRANSITION_TIMEOUT_S)

    tester.start_mission([(2000.0, 0.0)], FW_ALTITUDE_M)
    tester.wait_until_north_of(200.0, 120)
    tester.check_pusher_pushing()
    tester.fail_pusher()
    tester.check_pusher_stopped()
    check_fixed_wing_flight_stopped(tester)


@pytest.mark.vtol_pusher_failure
def test_working_pusher_in_fixed_wing_flight(
        tester_pusher_failure: AutopilotTesterPusherFailure) -> None:
    """Requirement 3: fixed-wing mission legs, which SIH flies above the check's throttle threshold,
    then a landing."""
    tester = tester_pusher_failure
    tester.request_ground_truth()
    tester.enable_fixed_wing_propulsion_check(MOTOR_CURRENT_MAX_A, PROPULSION_CURRENT_MIN_A)
    tester.take_off_with_pusher_assist(FW_ALTITUDE_M)
    tester.start_front_transition()
    tester.wait_until_fixedwing(TRANSITION_TIMEOUT_S)

    tester.start_mission([(400.0, 0.0), (400.0, 400.0), (0.0, 400.0)], FW_ALTITUDE_M)
    # Cruise, after the descent from the altitude overshoot of the transition lowered the throttle.
    tester.wait_until_north_of(200.0, 120)
    tester.check_pusher_output_above(GATE_OUTPUT, FW_DETECTION_TIME_S)
    tester.wait_until_disarmed(450)
    tester.check_no_quadchute()
    # A failsafe would have landed elsewhere.
    tester.check_mission_land_within(5.0)


def check_fixed_wing_flight_stopped(tester: AutopilotTesterPusherFailure) -> None:
    """The pusher failure quad-chute ends fixed-wing flight in time, and the vehicle lands in
    multicopter mode."""
    detection_delay_s = tester.wait_for_pusher_quadchute(FW_DETECTION_TIME_S)
    tester.wait_until_disarmed(240)
    # The state reported right at the quad-chute can still be fixed-wing.
    tester.check_only_multicopter_since(tester.pusher_failure_time_s() + detection_delay_s + 1)


def check_front_transition_stopped(tester: AutopilotTesterPusherFailure,
                                   transition_time_s: float) -> None:
    """The pusher failure quad-chute stops the front transition in time, the vehicle never reaches
    fixed-wing flight and lands in multicopter mode."""
    tester.wait_for_quadchute(transition_time_s, FT_DETECTION_TIME_S, PUSHER_QUADCHUTE_EVENT)
    tester.check_never_fixed_wing_since(transition_time_s, FT_DETECTION_TIME_S + 5)
    tester.wait_until_disarmed(240)
    tester.check_never_fixed_wing_since(transition_time_s, 0)
