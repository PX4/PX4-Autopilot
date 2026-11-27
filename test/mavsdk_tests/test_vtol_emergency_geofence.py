import pytest
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester_geofence import (
    ACTION_HOLD, CROSSING_DEPTH_M, RECOVERED_DEPTH_M, AutopilotTesterGeofence)

HOME = (0.0, 0.0)
ALTITUDE_M = 40.0
# Home is this far south of the northern edge of the fence, and far from its other edges.
FENCE_NORTH_M = 400.0
LOITER_RADIUS_M = 80.0
LOITER_TOLERANCE_M = 30.0
# Past the edge by the time the 5 Hz breach check sees it, at 10 m/s.
BREACH_TOLERANCE_M = 10.0
# Half the 10 m/s trim airspeed: past the far side of the loiter and flying back towards the fence.
INWARD_SPEED_M_S = 5.0
# Each position the loiter waits for comes once per orbit, and an orbit takes about 50 s.
ORBIT_TIMEOUT_S = 90.0


@pytest.mark.vtol
def test_emergency_geofence_hold_keeps_its_center_and_accepts_recovery(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """A fixed-wing pilot flies out of the fence, and GF_ACTION Hold loiters around the breach
    position.

    Half of that loiter is outside the fence: the Hold keeps its center while the loiter crosses
    the edge. A Go-to sent while the vehicle is outside brings it back, and crossing back in must
    not replace the Go-to target."""
    tester = tester_geofence
    tester.wait_until_ready()
    tester.store_home()
    tester.set_param_int("MIS_TKO_LAND_REQ", 0)
    tester.set_param_int("GF_ACTION", ACTION_HOLD)
    tester.set_param_float("NAV_LOITER_RAD", LOITER_RADIUS_M)
    tester.upload_inclusion_rectangle(-600.0, FENCE_NORTH_M, -500.0, 500.0)
    # The transition heads for the take-off waypoint, here north of Home. With the take-off
    # waypoint at Home, the heading would depend on centimeters of drift during the climb.
    tester.upload_vtol_mission([(150.0, 0.0), (250.0, 0.0)], ALTITUDE_M)
    tester.arm()
    tester.start_mission_raw_and_wait_for_sequence(0)
    tester.wait_until_fixedwing(60)

    # Centered sticks keep flying north in fixed-wing Position mode.
    tester.fly_in_position_mode_until_failsafe(FlightMode.HOLD, forward=0.0, timeout_s=90)
    with tester.keep_manual_control_alive():
        with tester.monitor_hold_position() as center:
            held_north_m = tester.get_coordinate_transformation().local_from_global(*center)[0]
            assert abs(held_north_m - FENCE_NORTH_M) < BREACH_TOLERANCE_M, \
                "Held {:.1f} m north of Home, not at the northern edge".format(held_north_m)
            # The loiter crosses the edge: out, back in, and out again until it flies back from
            # the far side, where the Go-to below is sent.
            tester.wait_until_outside_fence(CROSSING_DEPTH_M, ORBIT_TIMEOUT_S)
            tester.wait_until_inside_fence(CROSSING_DEPTH_M, ORBIT_TIMEOUT_S)
            tester.wait_for_position_ned(
                lambda s: s.velocity.north_m_s < -INWARD_SPEED_M_S
                and tester.current_fence_distance_m() < -CROSSING_DEPTH_M, ORBIT_TIMEOUT_S)

        tester.command_reposition(HOME, ALTITUDE_M)
        with tester.monitor_hold_position(expected_center=HOME):
            tester.wait_until_on_loiter(HOME, LOITER_RADIUS_M, LOITER_TOLERANCE_M, settle_s=3,
                                        timeout_s=120)
    tester.check_inside_fence(RECOVERED_DEPTH_M)
