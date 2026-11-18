import pytest
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import log
from autopilot_tester_geofence import ACTION_LAND, ACTION_WARNING, AutopilotTesterGeofence

MISSION_ALTITUDE_M = 40.0

# Home is 60 m inside the northern edge of the fence, and the vehicle first flies east along it.
FENCE_NORTH_M = 60.0
# The mission itself keeps the margin: its waypoints are at least 100 m from the fence.
WAYPOINTS = [(-40.0, 1500.0), (-300.0, 1500.0)]
# Wider than the distance of the vehicle on its way to the first waypoint, and narrower than the
# default loiter radius of 80 m.
MARGIN_M = 75.0


@pytest.mark.vtol
@pytest.mark.parametrize("geofence_action, final_flight_mode", [
    (ACTION_LAND, FlightMode.LAND),
    (ACTION_WARNING, FlightMode.HOLD),
], ids=["land", "warning"])
def test_more_severe_action_wins_after_contingency_geofence_hold(
        tester_geofence: AutopilotTesterGeofence, geofence_action: int,
        final_flight_mode: FlightMode) -> None:
    """A fixed-wing mission is held within the margin, and the loiter then crosses the emergency
    geofence.

    The margin is narrower than the loiter radius, so the Hold cannot stay inside the fence.
    With GF_ACTION Land, Land takes over. An emergency geofence that only warns leaves the Hold
    in place: the mission, which is still valid, must not resume."""
    tester = tester_geofence
    tester.wait_until_ready()
    tester.store_home()
    tester.set_param_int("MIS_TKO_LAND_REQ", 0)
    tester.set_param_int("GF_ACTION", geofence_action)
    tester.upload_inclusion_rectangle(-600.0, FENCE_NORTH_M, -300.0, 1900.0)
    tester.upload_vtol_mission(WAYPOINTS, MISSION_ALTITUDE_M)
    tester.arm()
    tester.start_mission_raw_and_wait_for_sequence(0)
    tester.wait_until_fixedwing(60)
    tester.wait_for_flight_mode(FlightMode.MISSION, 1)

    # The vehicle is closer to the fence than the margin that is set now.
    assert tester.home_distance_m() < 400.0, "The margin is set too late"
    tester.start_tracking()
    tester.set_param_float("GF_CONT_H_MARGIN", MARGIN_M)
    tester.wait_for_flight_mode(FlightMode.HOLD, 10)
    held_at_m = tester.min_fence_distance_m()
    log("Held {:.1f} m from the fence".format(held_at_m))
    assert 0.0 < held_at_m < MARGIN_M, "Not held inside the margin"

    if final_flight_mode == FlightMode.HOLD:
        # More than one orbit.
        tester.check_flight_mode_for(FlightMode.HOLD, 60)

    else:
        tester.wait_for_flight_mode(final_flight_mode, 120)

    closest_m = tester.min_fence_distance_m()
    log("Closest to the fence: {:.1f} m".format(closest_m))
    assert closest_m < 0.0, "The loiter did not cross the geofence"
