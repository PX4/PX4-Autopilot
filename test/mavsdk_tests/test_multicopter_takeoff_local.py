"""Local-frame takeoff on a GPS-less multicopter (sihsim_quadx_flow): MAV_CMD_NAV_TAKEOFF sent as
a COMMAND_INT with frame = MAV_FRAME_LOCAL_NED

Nothing here gives the estimator an origin (no SET_GPS_GLOBAL_ORIGIN), so the vehicle never gets a
*fused* global position -- tracked by failsafe_flags.global_position_invalid, which is what
Commander/Navigator key the local-frame takeoff dispatch on. That is not the same as having no
home position: Commander's HomePosition falls back directly to a raw GPS fix on vehicle_gnss
(independently of EKF2_SENS_EN, gated by EKF2_GPS_CTRL instead), so home can be valid even while
the fused global position stays unavailable. test_takeoff_local_home_from_raw_gps_without_fusion
covers that; test_takeoff_local_no_gps_sim is the only scenario here where nothing ever publishes a
GPS fix, so home stays invalid too.

test_takeoff_local_fuses_gps_after_climb_with_home_already_valid starts the same way, then switches
GPS fusion on in flight, to confirm an already-valid home does not change the global-position-gated
behaviour. Across this file, Commander's attempt to leave Takeoff for Hold is made exactly once,
right when the takeoff mission item finishes, and deliberately does not retry: a global position
that only shows up afterwards must not belatedly pull the vehicle into Hold

test_takeoff_local_reaches_hold_when_gps_already_fused is the control case: GPS fusion is already
active before the takeoff is even sent, so the one-shot attempt succeeds immediately, with minimal
drift.
test_takeoff_local_reaches_hold_when_gps_fused_mid_climb sits between that control and the
provoked-failure tests above: fusion starts mid-flight, but early enough -- partway through a much
higher climb -- to settle before the climb finishes, so the one-shot attempt succeeds the same way.

The takeoff command is sent *before* arming: arming normally needs a global position (whatever the
default mode requires), so sending the local-frame takeoff first makes Takeoff armable.

MAVSDK's own takeoff() always sends MAV_FRAME_GLOBAL_INT, so the command is built by hand here to
set MAV_FRAME_LOCAL_NED instead. wait_until_hovering() is skipped too: like is_armable, MAVSDK's
landed_state does not reliably update without a global position, so altitude (a plain local NED
reading) tracks the climb instead.
"""

import math

import pytest
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester_flow import AutopilotTesterFlow

MAV_CMD_NAV_TAKEOFF = 22
FRAME_LOCAL_NED = 1

# https://mavlink.io/en/messages/common.html#MAV_CMD_ESTIMATOR_SENSOR_ENABLE
MAV_CMD_ESTIMATOR_SENSOR_ENABLE = 43006
FUSION_SOURCE_GPS = 0

# mavlink_receiver.cpp scales a COMMAND_INT's int32 x/y by 1e4 in a local frame (1e7 is for
# global lat/lon).
LOCAL_FRAME_INT_SCALE = 1e4

TAKEOFF_ALTITUDE_M = 10.0

# Where the vehicle would loiter after the climb, were Hold ever able to run on this vehicle.
LOITER_NORTH_M = 0.0
LOITER_EAST_M = 0.0

# Time for Commander's post-takeoff mode-change attempt to run and settle once the takeoff
# altitude is reached.
MODE_SETTLE_S = 5

# Generous bound on acceleration while GPS fusion starts: ordinary flight (even climbing) keeps
# this small, so a real jump (e.g. an unhandled estimator reset) is a clear outlier. Acceleration
# rather than absolute speed, since speed alone is already non-zero during a climb for reasons
# unrelated to GPS fusion.
MAX_ACCEL_DURING_FUSION_START_M_S2 = 2.0

# Tight bound on horizontal displacement switching into Hold: Hold's freshly computed loiter
# target (Loiter::on_activation()) should coincide almost exactly with wherever the vehicle
# already was, since the switch happens before Takeoff's quasi-loiter has any time to move it.
MAX_HOLD_HANDOVER_DRIFT_M = 1.5

# Takeoff altitude for the mid-climb GPS-fusion test: much higher than TAKEOFF_ALTITUDE_M, to leave
# time for fusion to start and settle before the climb -- the "waypoint" completion looks at --
# finishes.
MIDCLIMB_TAKEOFF_ALTITUDE_M = 30.0


def _send_local_takeoff_command(tester: AutopilotTesterFlow) -> None:
    """Send the local-frame takeoff command before arming: makes Takeoff (local position only)
    the mode arming is checked against,
    z is NED (down-positive), so the target altitude is negative."""
    tester.send_command_int(
        MAV_CMD_NAV_TAKEOFF, frame=FRAME_LOCAL_NED,
        x=round(LOITER_NORTH_M * LOCAL_FRAME_INT_SCALE),
        y=round(LOITER_EAST_M * LOCAL_FRAME_INT_SCALE),
        z=-TAKEOFF_ALTITUDE_M)


@pytest.mark.takeoff_local
def test_takeoff_local_home_from_raw_gps_without_fusion(tester_flow: AutopilotTesterFlow) -> None:
    """This airframe still runs the simulated GPS (SENS_EN_GPSSIM 1); only EKF2_SENS_EN's GNSS
    bits are cleared, so EKF2 never fuses it. Commander's HomePosition falls back to the raw fix
    instead, so home ends up valid while the fused global position stays invalid for the whole
    flight."""
    tester = tester_flow

    _send_local_takeoff_command(tester)
    tester.wait_until_armable()

    # Home only latches once, while disarmed and landed, so it's already valid here
    tester.wait_until_home_position_valid(10)
    tester.check_global_position_invalid()

    tester.arm()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 20)

    tester.sleep_for(MODE_SETTLE_S)
    tester.wait_for_flight_mode(FlightMode.TAKEOFF, 1)
    tester.check_global_position_invalid()

    tester.land()
    tester.wait_until_disarmed()


@pytest.mark.takeoff_local
def test_takeoff_local_fuses_gps_after_climb_with_home_already_valid(
        tester_flow: AutopilotTesterFlow) -> None:
    """Same starting point as test_takeoff_local_home_from_raw_gps_without_fusion (home valid from
    the raw fix, fused global position not), but GPS fusion is switched on in flight with
    MAV_CMD_ESTIMATOR_SENSOR_ENABLE. Home must stay exactly as valid as it already was -- fusion
    starting must not move or invalidate it -- and the vehicle must stay in Takeoff regardless:
    Commander's one-shot attempt to leave Takeoff for Hold already ran and failed when the climb
    finished, Fusion starting must also not physically move the vehicle."""
    tester = tester_flow

    _send_local_takeoff_command(tester)
    tester.wait_until_armable()

    # Home only latches once, while disarmed and landed, so it's already valid here
    tester.wait_until_home_position_valid(10)
    tester.check_global_position_invalid()

    tester.arm()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 20)

    # Switch GPS fusion on in flight (EKF2_SENS_EN would only apply once disarmed).
    tester.send_command_long(
        MAV_CMD_ESTIMATOR_SENSOR_ENABLE, param1=FUSION_SOURCE_GPS, param2=0, param3=1)
    tester.wait_until_global_position_valid(15, max_accel_m_s2=MAX_ACCEL_DURING_FUSION_START_M_S2)

    # Home must still be exactly as valid as it already was -- fusion starting must not touch it.
    tester.wait_until_home_position_valid(1)

    # No retry: Commander's one-shot attempt to leave Takeoff for Hold already ran (and failed)
    # before fusion started, so the vehicle must stay in Takeoff for the rest of the flight.
    tester.sleep_for(MODE_SETTLE_S)
    tester.wait_for_flight_mode(FlightMode.TAKEOFF, 1)

    tester.land()
    tester.wait_until_disarmed()


@pytest.mark.takeoff_local_no_gps
def test_takeoff_local_no_gps_sim(tester_flow: AutopilotTesterFlow) -> None:
    """SENS_EN_GPSSIM 0 (this test's SITL config) means no publishing of a GPS fix: unlike
    test_takeoff_local_home_from_raw_gps_without_fusion, Commander's raw-GPS fallback has nothing
    to fall back to, so home stays invalid for the whole flight too, not just the fused global
    position."""
    tester = tester_flow

    _send_local_takeoff_command(tester)
    tester.wait_until_armable()
    tester.arm()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 20)

    tester.check_global_position_invalid()
    tester.check_home_position_invalid()

    tester.sleep_for(MODE_SETTLE_S)
    tester.wait_for_flight_mode(FlightMode.TAKEOFF, 1)
    tester.check_global_position_invalid()
    tester.check_home_position_invalid()

    tester.land()
    tester.wait_until_disarmed()


@pytest.mark.takeoff_local_gps_already_fused
def test_takeoff_local_reaches_hold_when_gps_already_fused(
        tester_flow: AutopilotTesterFlow) -> None:
    """EKF2_SENS_EN is restored to its default here, so GPS fusion is already active well before
    the takeoff is even sent -- a local-frame takeoff still makes sense to request regardless of
    whether a global position happens to be available, and climbs exactly like every other test
    here.

    The difference is at the top of the climb: canRun(AUTO_LOITER) is already true, so Commander's
    one-shot attempt to leave Takeoff for Hold succeeds immediately instead of leaving the vehicle
    stuck."""
    tester = tester_flow

    _send_local_takeoff_command(tester)
    tester.wait_until_armable()
    tester.arm()
    tester.wait_until_global_position_valid(10)
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 20)

    north_m, east_m, _down_m = tester.get_current_position_ned()

    # Hold must actually be reached here, quickly: the one-shot attempt succeeds immediately since
    # global_position_invalid was never true.
    tester.wait_for_flight_mode(FlightMode.HOLD, MODE_SETTLE_S)

    new_north_m, new_east_m, _new_down_m = tester.get_current_position_ned()
    drift_m = math.hypot(new_north_m - north_m, new_east_m - east_m)
    assert drift_m < MAX_HOLD_HANDOVER_DRIFT_M, \
        "Horizontal drift {:.2f} m switching into Hold, expected < {} m".format(
            drift_m, MAX_HOLD_HANDOVER_DRIFT_M)

    tester.land()
    tester.wait_until_disarmed()


@pytest.mark.takeoff_local
def test_takeoff_local_reaches_hold_when_gps_fused_mid_climb(
        tester_flow: AutopilotTesterFlow) -> None:
    """Same starting point as test_takeoff_local_home_from_raw_gps_without_fusion, but GPS fusion
    is switched on mid-climb -- not after it finishes. The climb goes to
    MIDCLIMB_TAKEOFF_ALTITUDE_M, much higher than elsewhere, to leave enough time for the fix to
    settle before reaching the altitude

    By the time the climb finishes, canRun(AUTO_LOITER) has already been true for a while, so
    Commander's one-shot attempt succeeds, same as the already-fused control case, but this time
    the global position only became available mid-flight."""

    tester = tester_flow

    tester.send_command_int(
        MAV_CMD_NAV_TAKEOFF, frame=FRAME_LOCAL_NED,
        x=0, y=0, z=-MIDCLIMB_TAKEOFF_ALTITUDE_M)
    tester.wait_until_armable()

    # Home only latches once, while disarmed and landed, so it's already valid here -- not
    # because of anything after arming.
    tester.wait_until_home_position_valid(10)
    tester.check_global_position_invalid()

    tester.arm()

    # Partway through the climb: neither right at liftoff nor anywhere near the top.
    tester.sleep_for(10)

    # Switch GPS fusion on in flight (EKF2_SENS_EN would only apply once disarmed).
    tester.send_command_long(
        MAV_CMD_ESTIMATOR_SENSOR_ENABLE, param1=FUSION_SOURCE_GPS, param2=0, param3=1)
    tester.wait_until_global_position_valid(15, max_accel_m_s2=MAX_ACCEL_DURING_FUSION_START_M_S2)

    # Confirm this happened mid-climb, not coincidentally at the top; also the "before" snapshot
    # for the drift check below (north = east = 0 the whole flight, so position shouldn't move
    # either way).
    north_m, east_m, mid_down_m = tester.get_current_position_ned()
    mid_climb_altitude_m = -mid_down_m
    assert mid_climb_altitude_m < MIDCLIMB_TAKEOFF_ALTITUDE_M - 10.0, \
        "Already at {:.1f} m of {} m when the global position became valid, too close to the " \
        "top to show this happening mid-climb".format(
            mid_climb_altitude_m, MIDCLIMB_TAKEOFF_ALTITUDE_M)

    # Hold activates once Navigator's own looser altitude check passes, not necessarily within
    # this test's tighter wait_until_altitude tolerance, and then holds at whatever altitude that
    # was -- so wait for the mode switch itself, generously, rather than an exact altitude match.
    tester.wait_for_flight_mode(FlightMode.HOLD, 90)

    new_north_m, new_east_m, _new_down_m = tester.get_current_position_ned()
    drift_m = math.hypot(new_north_m - north_m, new_east_m - east_m)
    assert drift_m < MAX_HOLD_HANDOVER_DRIFT_M, \
        "Horizontal drift {:.2f} m switching into Hold, expected < {} m".format(
            drift_m, MAX_HOLD_HANDOVER_DRIFT_M)

    tester.land()
    tester.wait_until_disarmed()
