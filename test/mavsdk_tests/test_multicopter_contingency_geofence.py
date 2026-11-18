import pytest
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import MissionOptions, log
from autopilot_tester_geofence import AutopilotTesterGeofence
from autopilot_tester_loiter import horizontal_distance

TAKEOFF_ALTITUDE_M = 10.0

# At 5 m/s the vehicle is past a limit by the time the 5 Hz check sees it, and then it has to brake.
OVERSHOOT_M = 20.0
# Between the estimate that triggers and the ground truth that is measured.
POSITION_TOLERANCE_M = 2.0


def take_off(tester: AutopilotTesterGeofence) -> None:
    tester.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 30)


@pytest.mark.multicopter
@pytest.mark.parametrize("margin_m", [0.0, 40.0])
def test_pilot_is_held_at_geofence_or_margin(tester_geofence: AutopilotTesterGeofence,
                                             margin_m: float) -> None:
    """A pilot flies towards the fence in Position mode.

    Without a margin the vehicle is only held once it has crossed the emergency geofence. With a
    margin it is held when it comes that close, and never reaches it."""
    tester = tester_geofence
    tester.wait_until_ready()
    tester.store_home()
    tester.upload_inclusion_rectangle(-80.0, 80.0, -80.0, 80.0)
    tester.set_param_float("GF_CONT_H_MARGIN", margin_m)
    take_off(tester)

    tester.start_tracking()
    tester.fly_in_position_mode_until_hold(forward=0.5, throttle=0.5, timeout_s=60)
    tester.keep_sticks_centered_for(8)
    tester.wait_for_flight_mode(FlightMode.HOLD, 1)

    closest_m = tester.min_fence_distance_m()
    log("Closest to the fence: {:.1f} m".format(closest_m))
    assert margin_m - OVERSHOOT_M < closest_m < margin_m + POSITION_TOLERANCE_M, \
        "Came within {:.1f} m of the fence with a margin of {} m".format(closest_m, margin_m)


@pytest.mark.multicopter
def test_mission_is_held_at_contingency_geofence_home_distance(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """GF_PRE_MAX_HDIST holds a mission before it reaches GF_MAX_HOR_DIST."""
    tester = tester_geofence
    tester.wait_until_ready()
    tester.store_home()
    tester.set_param_float("GF_MAX_HOR_DIST", 300.0)
    # Straight north, to 240 m.
    tester.prepare_straight_mission(MissionOptions(leg_length_m=60.0, relative_altitude_m=15.0))
    tester.arm()
    tester.start_and_wait_for_mission_sequence(1)

    # A mission that is being flown keeps its verdict, so the limit can be set behind it.
    tester.set_param_float("GF_PRE_MAX_HDIST", 150.0)
    assert tester.home_distance_m() < 120.0, "The limit was set too late"
    tester.start_tracking()
    tester.wait_for_flight_mode(FlightMode.HOLD, 60)
    # The mission does not resume.
    tester.check_flight_mode_for(FlightMode.HOLD, 10)

    furthest_m = tester.max_home_distance_m()
    log("Furthest from Home: {:.1f} m".format(furthest_m))
    assert 150.0 - POSITION_TOLERANCE_M < furthest_m < 150.0 + OVERSHOOT_M, \
        "Went {:.1f} m from Home".format(furthest_m)


@pytest.mark.multicopter
def test_pilot_is_held_at_contingency_geofence_altitude(tester_geofence: AutopilotTesterGeofence) -> None:
    """GF_PRE_MAX_VDIST holds a climbing pilot before GF_MAX_VER_DIST is reached."""
    tester = tester_geofence
    tester.wait_until_ready()
    tester.store_home()
    tester.set_param_float("GF_MAX_VER_DIST", 45.0)
    tester.set_param_float("GF_PRE_MAX_VDIST", 25.0)
    take_off(tester)

    tester.start_tracking()
    tester.fly_in_position_mode_until_hold(forward=0.0, throttle=1.0, timeout_s=60)
    tester.keep_sticks_centered_for(8)
    tester.wait_for_flight_mode(FlightMode.HOLD, 1)

    highest_m = tester.max_relative_altitude_m()
    log("Highest above Home: {:.1f} m".format(highest_m))
    assert 25.0 - POSITION_TOLERANCE_M < highest_m < 25.0 + OVERSHOOT_M, \
        "Climbed to {:.1f} m".format(highest_m)


@pytest.mark.multicopter
def test_contingency_geofence_refuses_arming_missions_and_targets(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """Within the margin arming is denied, and a mission or a Go-to that leads there is refused."""
    tester = tester_geofence
    tester.wait_until_ready()
    tester.store_home()
    # Home is 30 m from each edge.
    tester.upload_inclusion_rectangle(-30.0, 30.0, -30.0, 30.0)

    tester.set_param_float("GF_CONT_H_MARGIN", 40.0)
    tester.sleep_for(2)
    assert tester.arming_is_denied(), "Armed within the contingency geofence margin"

    tester.set_param_float("GF_CONT_H_MARGIN", 10.0)
    tester.sleep_for(2)
    options = MissionOptions(relative_altitude_m=TAKEOFF_ALTITUDE_M)

    # 5 m from the northern edge: the mission is rejected, so Mission mode cannot be armed.
    tester.upload_mission_to(25.0, options)
    tester.select_mission_mode()
    assert tester.arming_is_denied(), "Armed with a mission into the margin"

    # 15 m from the northern edge.
    tester.upload_mission_to(15.0, options)
    tester.arm()
    tester.execute_mission()

    # The vehicle now holds at the last waypoint. A target within the margin leaves it there.
    tester.command_reposition((25.0, 0.0), TAKEOFF_ALTITUDE_M)
    tester.check_stays_within((15.0, 0.0), 3.0, 8)

    tester.command_reposition((-15.0, 0.0), TAKEOFF_ALTITUDE_M)
    tester.wait_for_position_ned(lambda s: horizontal_distance(s, (-15.0, 0.0)) < 2.0, 30)
