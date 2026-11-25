import pytest
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import MissionOptions, log
from autopilot_tester_geofence import ACTION_RETURN, AutopilotTesterGeofence
from autopilot_tester_loiter import horizontal_distance

HOME = (0.0, 0.0)
TAKEOFF_ALTITUDE_M = 10.0
MISSION_ALTITUDE_M = 15.0

# At 5 m/s the vehicle is past a limit by the time the 5 Hz check sees it, and then it has to brake.
OVERSHOOT_M = 20.0
# Between the estimate that triggers and the ground truth that is measured.
POSITION_TOLERANCE_M = 2.0


@pytest.mark.multicopter
def test_pilot_is_held_at_contingency_geofence_margin(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """A pilot flies towards the fence in Position mode. The vehicle is held when it comes within
    the margin, and never reaches the fence. A Go-to back out of the margin is flown, and one into
    it is refused.

    Without a margin it is only held once it has crossed the fence, see
    test_multicopter_emergency_geofence.py."""
    tester = tester_geofence
    tester.wait_until_ready()
    tester.store_home()
    tester.upload_inclusion_rectangle(-80.0, 80.0, -80.0, 80.0)
    margin_m = 40.0
    tester.set_param_float("GF_CONT_H_MARGIN", margin_m)
    tester.arm_and_take_off(TAKEOFF_ALTITUDE_M)

    tester.start_tracking()
    tester.fly_in_position_mode_until_failsafe(FlightMode.HOLD)
    with tester.keep_manual_control_alive():
        with tester.monitor_hold_position():
            tester.wait_until_speed_lower_than(1.0, 10)
        # Home is out of the margin, so this Go-to is flown although the vehicle is still in it.
        clearance_m = tester.current_fence_distance_m()
        assert 0.0 < clearance_m < margin_m, "Recovery started {:.1f} m from the fence".format(
            clearance_m)
        tester.command_reposition(HOME, TAKEOFF_ALTITUDE_M)
        with tester.monitor_hold_position(expected_center=HOME):
            tester.wait_for_position_ned(lambda s: horizontal_distance(s, HOME) < 3.0, 30)
            # In the margin, 10 m from the fence: refused, so the Hold stays at Home.
            tester.check_reposition_rejected((70.0, 0.0), TAKEOFF_ALTITUDE_M,
                                             "navigator_reposition_outside_cont_geofence")

    closest_m = tester.min_fence_distance_m()
    log("Closest to the fence: {:.1f} m".format(closest_m))
    assert margin_m - OVERSHOOT_M < closest_m < margin_m + POSITION_TOLERANCE_M, \
        "Came within {:.1f} m of the fence with a margin of {} m".format(closest_m, margin_m)


@pytest.mark.multicopter
def test_mission_is_held_at_contingency_geofence_home_distance(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """GF_CONT_H_MARGIN holds a mission that distance before it reaches GF_MAX_HOR_DIST."""
    tester = tester_geofence
    tester.wait_until_ready()
    tester.store_home()
    tester.set_param_float("GF_MAX_HOR_DIST", 300.0)
    # Straight north, to 240 m.
    tester.prepare_straight_mission(
        MissionOptions(leg_length_m=60.0, relative_altitude_m=MISSION_ALTITUDE_M))
    tester.arm()
    tester.start_and_wait_for_mission_sequence(1)

    # A mission that is being flown keeps its verdict, so the margin can be set behind it.
    tester.set_param_float("GF_CONT_H_MARGIN", 150.0)
    assert tester.home_distance_m() < 120.0, "The margin was set too late"
    tester.start_tracking()
    tester.wait_for_flight_mode(FlightMode.HOLD, 60)
    # The mission does not resume.
    tester.check_flight_mode_for(FlightMode.HOLD, 10)

    furthest_m = tester.max_home_distance_m()
    log("Furthest from Home: {:.1f} m".format(furthest_m))
    assert 150.0 - POSITION_TOLERANCE_M < furthest_m < 150.0 + OVERSHOOT_M, \
        "Went {:.1f} m from Home".format(furthest_m)


@pytest.mark.multicopter
def test_contingency_geofence_return_lands_at_home(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """With GF_CONT_ACTION Return a mission that comes within the margin flies back and lands.

    The Return starts within the margin, so it first has to fly out of it."""
    tester = tester_geofence
    tester.wait_until_ready()
    tester.store_home()
    tester.set_param_int("GF_CONT_ACTION", ACTION_RETURN)
    tester.set_rtl_altitude(MISSION_ALTITUDE_M)
    # Home is 300 m from each edge.
    tester.upload_inclusion_rectangle(-300.0, 300.0, -300.0, 300.0)
    # Straight north, to 240 m.
    tester.prepare_straight_mission(
        MissionOptions(leg_length_m=60.0, relative_altitude_m=MISSION_ALTITUDE_M))
    tester.arm()
    tester.start_and_wait_for_mission_sequence(1)

    # A mission that is being flown keeps its verdict, so the margin can be set behind it.
    tester.set_param_float("GF_CONT_H_MARGIN", 150.0)
    assert tester.home_distance_m() < 120.0, "The margin was set too late"
    tester.start_tracking()
    tester.wait_for_flight_mode(FlightMode.RETURN_TO_LAUNCH, 60)
    tester.wait_until_disarmed(180)
    tester.check_home_within(5.0)

    closest_m = tester.min_fence_distance_m()
    log("Closest to the fence: {:.1f} m".format(closest_m))
    assert 150.0 - OVERSHOOT_M < closest_m < 150.0 + POSITION_TOLERANCE_M, \
        "Came within {:.1f} m of the fence with a margin of 150 m".format(closest_m)
