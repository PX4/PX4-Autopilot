"""Emergency geofence (GF_ACTION) flights. The contingency geofence is off by default.

The breach geometry and the action policy are unit tested in Navigator and Commander. These flights
check the whole chain: an uploaded fence, a real crossing and the flight mode it leads to, the
recovery, and a refused Go-to.
"""

import pytest
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import log
from autopilot_tester_geofence import (
    ACTION_HOLD, ACTION_LAND, ACTION_RETURN, ACTION_WARNING, AutopilotTesterGeofence)
from geofence_geometry import FenceKind, FenceShape, rectangle

HOME = (0.0, 0.0)
# Home is in the middle of a square inclusion fence, this far from each edge.
FENCE_DISTANCE_M = 80.0
INCLUSION = rectangle(-FENCE_DISTANCE_M, FENCE_DISTANCE_M, -FENCE_DISTANCE_M, FENCE_DISTANCE_M)
TAKEOFF_ALTITUDE_M = 20.0
# Past the fence by the time the 5 Hz breach check sees it, and then braking.
OVERSHOOT_M = 20.0
# Between the estimate that triggers and the ground truth that is measured.
POSITION_TOLERANCE_M = 2.0

# GF_SOURCE: check the raw GNSS position instead of the estimate.
GF_SOURCE_GNSS = 1


def prepare(tester: AutopilotTesterGeofence, action: int) -> None:
    tester.wait_until_ready()
    tester.store_home()
    tester.set_param_int("GF_ACTION", action)
    # Return and Land start at the breach, without the default 5 s Hold before them. The delay is
    # unit tested in Commander.
    tester.set_param_float("COM_FAIL_ACT_T", 0.0)


def upload_inclusion(tester: AutopilotTesterGeofence) -> None:
    tester.upload_fence([FenceShape(FenceKind.POLYGON_INCLUSION, vertices=INCLUSION)])


def take_off(tester: AutopilotTesterGeofence, altitude_m: float = TAKEOFF_ALTITUDE_M) -> None:
    tester.arm_and_take_off(altitude_m)
    tester.start_tracking()


def check_acted_at_the_fence(tester: AutopilotTesterGeofence) -> None:
    """Call it once the vehicle stopped, so that the overshoot counts."""
    closest_m = tester.min_fence_distance_m()
    log("Closest to the fence: {:.1f} m".format(closest_m))
    assert -OVERSHOOT_M < closest_m < POSITION_TOLERANCE_M, \
        "Closest to the fence {:.1f} m, expected at the fence".format(closest_m)


def breach_recover_and_breach_again(tester: AutopilotTesterGeofence, action: int,
                                    flight_mode: FlightMode) -> None:
    """A pilot flies out of the fence and GF_ACTION takes over. A Go-to sent while the vehicle is
    still outside brings it back, and crossing back in must not replace the Go-to target. Then the
    pilot flies out again, until GF_ACTION takes over again."""
    prepare(tester, action)
    upload_inclusion(tester)
    take_off(tester)
    tester.fly_in_position_mode_until_failsafe(flight_mode)
    tester.recover_with_reposition(HOME, TAKEOFF_ALTITUDE_M)
    check_acted_at_the_fence(tester)

    tester.reset_tracking_extremes()
    # The Go-to can turn the vehicle, so it may fly out across another edge of the square.
    tester.fly_in_position_mode_until_failsafe(flight_mode)


@pytest.mark.multicopter
def test_emergency_geofence_holds_again_after_a_goto_recovery(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """See breach_recover_and_breach_again(). The second Hold is where the vehicle crossed the
    fence again, not at the Go-to target, and a Go-to outside the fence is refused."""
    tester = tester_geofence
    breach_recover_and_breach_again(tester, ACTION_HOLD, FlightMode.HOLD)
    with tester.keep_manual_control_alive(), tester.monitor_hold_position() as center:
        # Outside the fence whichever edge the vehicle crossed: the Hold stays where it is.
        tester.command_reposition((2 * FENCE_DISTANCE_M, 0.0), TAKEOFF_ALTITUDE_M)
        tester.sleep_for(3)
    check_acted_at_the_fence(tester)
    held_at_m = tester.fence_distance_m(*center)
    assert held_at_m < POSITION_TOLERANCE_M, "Held {:.1f} m inside the fence".format(held_at_m)


@pytest.mark.multicopter
def test_emergency_geofence_lands_again_after_a_goto_recovery(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """See breach_recover_and_breach_again(). The second Land ends outside the fence, where the
    breach denies arming."""
    tester = tester_geofence
    breach_recover_and_breach_again(tester, ACTION_LAND, FlightMode.LAND)
    with tester.keep_manual_control_alive():
        tester.wait_until_disarmed(90)
    check_acted_at_the_fence(tester)
    landed_at_m = tester.current_fence_distance_m()
    assert landed_at_m < 0.0, "Landed {:.1f} m inside the fence".format(landed_at_m)

    # Unlike Position mode, Hold arms without manual control, which is no longer sent.
    tester.hold()
    tester.wait_for_flight_mode(FlightMode.HOLD, 5)
    with tester.observe_event("Arming denied", "Emergency geofence breach") as denied:
        tester.check_arming_is_denied()
        assert tester.poll_condition_with_timeout(denied.is_set, 5), \
            "Arming was not denied for the emergency geofence breach"


@pytest.mark.multicopter
def test_emergency_geofence_home_distance_triggers_return(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """Flying beyond GF_MAX_HOR_DIST triggers Return, which lands at Home."""
    tester = tester_geofence
    prepare(tester, ACTION_RETURN)
    tester.set_param_float("GF_MAX_HOR_DIST", FENCE_DISTANCE_M)
    tester.set_rtl_altitude(TAKEOFF_ALTITUDE_M)
    take_off(tester)
    tester.fly_in_position_mode_until_failsafe(FlightMode.RETURN_TO_LAUNCH)
    with tester.keep_manual_control_alive():
        tester.wait_until_disarmed(90)
    tester.check_home_within(5.0)

    furthest_m = tester.max_home_distance_m()
    log("Furthest from Home: {:.1f} m".format(furthest_m))
    assert FENCE_DISTANCE_M - POSITION_TOLERANCE_M < furthest_m < FENCE_DISTANCE_M + OVERSHOOT_M, \
        "Went {:.1f} m from Home with a limit of {} m".format(furthest_m, FENCE_DISTANCE_M)


@pytest.mark.multicopter
def test_emergency_geofence_altitude_triggers_hold(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """Climbing through GF_MAX_VER_DIST holds the vehicle at that altitude, still above Home."""
    tester = tester_geofence
    prepare(tester, ACTION_HOLD)
    max_altitude_m = 20.0
    tester.set_param_float("GF_MAX_VER_DIST", max_altitude_m)
    # Take off below the limit and climb through it.
    take_off(tester, max_altitude_m - 10.0)
    tester.fly_in_position_mode_until_failsafe(FlightMode.HOLD, forward=0.0, throttle=0.8,
                                               timeout_s=30)
    with tester.keep_manual_control_alive(), tester.monitor_hold_position():
        tester.check_holds_altitude(max_altitude_m, 5.0, 5)
    tester.check_home_within(5.0)


@pytest.mark.multicopter
def test_emergency_exclusion_circle_triggers_hold_with_gnss_source(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """With GF_SOURCE GNSS, flying into an exclusion circle holds the vehicle at its edge.

    SIH estimate and GNSS agree, so this only shows that the GNSS source works in the full system.
    Which source is used is unit tested in Navigator."""
    tester = tester_geofence
    prepare(tester, ACTION_HOLD)
    tester.set_param_int("GF_SOURCE", GF_SOURCE_GNSS)
    # The southern edge of the circle is FENCE_DISTANCE_M north of Home.
    radius_m = 30.0
    tester.upload_fence([FenceShape(FenceKind.CIRCLE_EXCLUSION,
                                    center=(FENCE_DISTANCE_M + radius_m, 0.0), radius_m=radius_m)])
    take_off(tester)
    tester.fly_in_position_mode_until_failsafe(FlightMode.HOLD)
    with tester.keep_manual_control_alive(), tester.monitor_hold_position():
        tester.sleep_for(3)
    check_acted_at_the_fence(tester)


@pytest.mark.multicopter
def test_emergency_geofence_warning_keeps_position_mode(
        tester_geofence: AutopilotTesterGeofence) -> None:
    """With GF_ACTION Warning a breach is reported, and the pilot keeps flying in Position mode."""
    tester = tester_geofence
    prepare(tester, ACTION_WARNING)
    upload_inclusion(tester)
    take_off(tester)
    with tester.observe_event("Failsafe warning", "Emergency geofence breach") as warning:
        tester.fly_in_position_mode_until(
            lambda: warning.is_set()
            and tester.current_fence_distance_m() < -POSITION_TOLERANCE_M)
    with tester.keep_manual_control_alive():
        tester.check_flight_mode_for(FlightMode.POSCTL, 2)
