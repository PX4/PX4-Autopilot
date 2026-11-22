"""Pusher failure detection of standard VTOLs (VT_EF_TIME, VT_EF_FT_ACC).

What the detection must achieve, whatever the detector:
1. A pusher that failed before or early in a front transition triggers the pusher failure
   quad-chute within FT_DETECTION_TIME_S of the transition start, with or without airspeed
   sensor. The vehicle never reaches fixed-wing flight and lands in multicopter mode.
2. A pusher that fails while the vehicle flies forward with pusher assist (VT_FWD_THRUST_EN) in
   multicopter mode triggers the pusher failure quad-chute within DETECTION_TIME_S. The vehicle
   then stays in multicopter mode and lands.
3. A working pusher triggers no quad-chute, in multicopter flight and in front transitions.

Accepted, not required: a headwind stronger than the pusher may be reported as a pusher failure,
since the vehicle cannot make progress with pusher assist anyway.
Known gaps of the VT_EF_* detector: a tailwind that carries the vehicle within VT_EF_VEL_ERR of
its velocity setpoint hides the failure, a pusher that fails in hover is only detected on the
next pusher-assisted leg or at the start of the next front transition, and a pusher that fails
after the transition check passed (about 3.4 s into the transition) is not detected.

The detection times are logged, so detectors can be compared with the same flights:
    test/mavsdk_tests/mavsdk_test_runner.py --speed-factor 10 --model standard_vtol \\
        --case '*pusher*' test/mavsdk_tests/configs/sih-sitl.json
SIH has constant wind only and a 0.2 kg vehicle, so the flights check behavior, not tuning.
"""

import pytest

from autopilot_tester_pusher_failure import (PUSHER_QUADCHUTE_EVENT,
                                             AutopilotTesterPusherFailure, FixedWingReached,
                                             Wind)

ALTITUDE_M = 20.0
SETTLE_S = 10.0
DETECTION_TIME_S = 10.0
# A failed pusher has to stop the front transition before the open-loop transition would complete
# without airspeed (VT_F_TR_OL_TM default).
FT_DETECTION_TIME_S = 6.0
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
    tester.start_multicopter_mission([(300.0, 0.0)], ALTITUDE_M)
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
    tester.start_multicopter_mission([(100.0, 0.0), (100.0, 60.0), (0.0, 60.0)], ALTITUDE_M)
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
    tester.start_multicopter_mission([(300.0, 0.0)], ALTITUDE_M)
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
@pytest.mark.xfail(
    strict=True, raises=FixedWingReached,
    reason="the transition check ends once the pusher accelerated the vehicle, so a later failure "
           "completes the open-loop transition (VT_F_TR_OL_TM)")
def test_pusher_failure_late_in_front_transition(
        tester_pusher_failure: AutopilotTesterPusherFailure) -> None:
    """Known gap: without airspeed, the pusher stops after it accelerated the vehicle, but before
    the open-loop transition completes."""
    tester = tester_pusher_failure
    tester.take_off_with_pusher_assist(ALTITUDE_M, use_airspeed=False)

    transition_time_s = tester.start_front_transition()
    # The check decides 3.4 s into the transition, at the default VT_PSHER_SLEW.
    tester.sleep_for(5)
    tester.fail_pusher()
    tester.check_pusher_stopped()
    tester.check_never_fixed_wing_since(transition_time_s, FT_DETECTION_TIME_S + 5)


def check_front_transition_stopped(tester: AutopilotTesterPusherFailure,
                                   transition_time_s: float) -> None:
    """The pusher failure quad-chute stops the front transition in time, the vehicle never reaches
    fixed-wing flight and lands in multicopter mode."""
    tester.wait_for_quadchute(transition_time_s, FT_DETECTION_TIME_S, PUSHER_QUADCHUTE_EVENT)
    tester.check_never_fixed_wing_since(transition_time_s, FT_DETECTION_TIME_S + 5)
    tester.wait_until_disarmed(240)
    tester.check_never_fixed_wing_since(transition_time_s, 0)
