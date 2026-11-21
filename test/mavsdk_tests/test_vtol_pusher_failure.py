"""Pusher failure detection of standard VTOLs with pusher assist (VT_FWD_THRUST_EN).

What the detection must achieve, whatever the detector:
1. A vehicle with a failed pusher never reaches fixed-wing flight.
2. A pusher that fails while the vehicle flies forward with pusher assist in multicopter mode
   triggers the pusher failure quad-chute within DETECTION_TIME_S. The vehicle then stays in
   multicopter mode and lands.
3. A working pusher triggers no quad-chute.

Accepted, not required: a headwind stronger than the pusher may be reported as a pusher failure,
since the vehicle cannot make progress with pusher assist anyway.
Known gaps of the VT_EF_* detector: a tailwind that carries the vehicle within VT_EF_VEL_ERR of
its velocity setpoint hides the failure, and a pusher that fails in hover is only detected on the
next pusher-assisted leg or at the start of the next front transition.

The detection times are logged, so detectors can be compared with the same flights:
    test/mavsdk_tests/mavsdk_test_runner.py --speed-factor 10 --model standard_vtol \\
        --case '*pusher*' test/mavsdk_tests/configs/sih-sitl.json
SIH has constant wind only and a 0.2 kg vehicle, so the flights check behavior, not tuning.
"""

import pytest

from autopilot_tester_pusher_failure import AutopilotTesterPusherFailure, Wind

ALTITUDE_M = 20.0
SETTLE_S = 10.0
DETECTION_TIME_S = 10.0
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

    transition_time_s = tester.time_s()
    tester.transition_to_fixedwing()
    tester.wait_until_transition_started(transition_time_s, 5)
    tester.sleep_for(1)
    tester.check_pusher_stopped()
    tester.check_never_fixed_wing_since(transition_time_s, TRANSITION_TIMEOUT_S + 5)

    tester.wait_for_quadchute(transition_time_s, TRANSITION_TIMEOUT_S + 5)
    tester.wait_until_multicopter(10)
