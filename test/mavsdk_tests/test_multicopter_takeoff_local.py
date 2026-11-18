"""Local-frame takeoff on a GPS-less multicopter (sihsim_quadx_flow), commanded with
MAV_CMD_NAV_TAKEOFF in MAV_FRAME_LOCAL_NED, with no global position ever available.

PX4 no longer has a dedicated local-frame takeoff command: a local-frame takeoff is requested by
sending the ordinary MAV_CMD_NAV_TAKEOFF as a COMMAND_INT with frame = MAV_FRAME_LOCAL_NED,
rather than by command id. VEHICLE_CMD_NAV_TAKEOFF_LOCAL still exists as a vehicle_command/MAVLink
enum value, but PX4 no longer dispatches on it.

Unlike test_multicopter_optical_flow.py, nothing here ever gives the estimator an origin (no
SET_GPS_GLOBAL_ORIGIN), so the vehicle never gets a *fused* global position, not even a coarse
one. That is tracked by failsafe_flags.global_position_invalid (derived from vehicle_global_position
alone), which is what Commander and Navigator actually key the local-frame takeoff dispatch on.

It is not the same as having no home position: whenever a raw GPS fix is present on the
vehicle_gnss topic, Commander's HomePosition falls back to it directly (independently of
EKF2_SENS_EN, gated instead by EKF2_GPS_CTRL) and sets a valid home lat/lon even though the fused
global position stays unavailable. test_takeoff_local_home_from_raw_gps_without_fusion below
exercises exactly this, and test_takeoff_local_no_gps_sim is the one scenario here where home
stays unset too, because nothing ever publishes a GPS fix at all.

test_takeoff_local_fuses_gps_after_climb_with_home_already_valid starts the same way (home valid
from the raw fix, global position not) and then switches GPS fusion on in flight, to check that
having a valid home already does not change how the global-position-gated behaviour plays out.

test_takeoff_local_gps_enabled_after_climb goes further still: GPS and its fusion into the EKF
both only appear once the climb is already finished. That exposes a home position asymmetry worth
spelling out: HomePosition only ever latches a first-time home while disarmed and landed, so a GPS
fix that shows up later in flight never moves it, however good the fix is -- only the fused global
position can still come up mid-flight, from EKF2 alone. Both tests also pin down that this does
not change where Commander leaves the vehicle: its attempt to leave Takeoff for Hold is made once,
right when the takeoff mission item finishes, and deliberately does not retry -- a global position
that only shows up afterwards must not belatedly pull the vehicle into Hold, so it must stay in
Takeoff for the rest of the flight regardless, by design, not as a limitation. Both tests do still
check that the vehicle is not disturbed by the GPS fusion starting, independently of any mode
switch.

All of the above deliberately provoke Commander's one-shot Takeoff-to-Hold attempt to fail.
test_takeoff_local_reaches_hold_when_gps_already_fused is the control for that: GPS fusion is
already active before the takeoff is even sent, so the attempt succeeds immediately when the
climb finishes, and the switch into Hold happens with minimal drift precisely because it happens
before Takeoff's own quasi-loiter has had any time to move the vehicle anywhere.

test_takeoff_local_reaches_hold_when_gps_fused_mid_climb sits between that control and the
provoked-failure tests: GPS fusion is switched on mid-flight, like the failures, but early enough
-- partway through a much higher climb -- that it has already settled by the time the climb
finishes, so the one-shot attempt still succeeds like the control does. It also commands no
horizontal target at all (north = east = 0), so any drift measured switching into Hold can only
come from the frame switch itself, not from a horizontal transit towards a commanded point.

Arming on this vehicle normally needs a global position (whatever mode is selected by default
requires it), so the takeoff command is sent *before* arming: a local-frame takeoff only needs a
local position, and sending it first makes Takeoff (not the default mode) the one arming is checked
against.

The takeoff's x/y is where the vehicle would loiter once the climb is done, not a horizontal
target flown to during the climb itself (Takeoff only climbs; FlightTaskAuto only starts moving
towards x/y once a Loiter/Hold setpoint takes over), and PX4's own "waypoint reached" check for a
local takeoff looks at altitude alone. So once the altitude is reached, Commander tries to go on
to that Hold, which needs a global position this vehicle never has, so it must leave the vehicle
holding where Takeoff left it (still above the takeoff point, not at x/y) instead of forcing a
Hold it cannot execute and getting stuck failing to run it on every subsequent cycle.

MAVSDK's own takeoff() always sends a plain MAV_CMD_NAV_TAKEOFF with MAV_FRAME_GLOBAL_INT, so the
command is built and sent by hand here instead, to set MAV_FRAME_LOCAL_NED. wait_until_hovering()
is skipped too: like is_armable, MAVSDK's landed_state (EXTENDED_SYS_STATE) does not reliably
update without a global position, so altitude (a plain local NED reading) is what the climb is
tracked by instead.
"""

import math

import pytest
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester_flow import AutopilotTesterFlow

MAV_CMD_NAV_TAKEOFF = 22
MAV_FRAME_LOCAL_NED = 1
# ArduPilot's convention for a local-frame takeoff; PX4 only accepts MAV_FRAME_LOCAL_NED (see
# mavlink_receiver.cpp's command_has_location() frame check) and must reject this one rather than
# silently feed its x/y, scaled as local meters, to Commander/Navigator as if they were global
# lat/lon degrees.
MAV_FRAME_LOCAL_OFFSET_NED = 7

# https://mavlink.io/en/messages/common.html#MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME
MAV_RESULT_UNSUPPORTED_MAV_FRAME = 9

# https://mavlink.io/en/messages/common.html#MAV_CMD_ESTIMATOR_SENSOR_ENABLE
MAV_CMD_ESTIMATOR_SENSOR_ENABLE = 43006
FUSION_SOURCE_GPS = 0

# mavlink_receiver.cpp scales a COMMAND_INT's int32 x/y by 1e4 in a local frame (1e7 is for
# global lat/lon).
LOCAL_FRAME_INT_SCALE = 1e4

TAKEOFF_ALTITUDE_M = 10.0

# Where the vehicle would loiter after the climb, were Hold ever able to run on this vehicle.
LOITER_NORTH_M = 5.0
LOITER_EAST_M = 5.0

# Time for Commander's post-takeoff mode-change attempt to run and settle once the takeoff
# altitude is reached.
MODE_SETTLE_S = 5

# Generous bound on horizontal+vertical speed while GPS fusion starts: holding in Takeoff should
# stay close to stationary, so a real jump (e.g. an unhandled estimator reset) is a clear outlier
# well below what would trip this, while ordinary hold noise comfortably will not.
MAX_SPEED_DURING_FUSION_START_M_S = 1.0

# Tight bound on net horizontal displacement switching into Hold when the global position was
# already there well before the climb finished: Hold's freshly computed loiter target
# (Loiter::on_activation()) should coincide almost exactly with wherever the vehicle already was,
# since the switch happens before Takeoff's own quasi-loiter has any time to move the vehicle.
MAX_HOLD_HANDOVER_DRIFT_M = 1.5

# Takeoff altitude for test_takeoff_local_reaches_hold_when_gps_fused_mid_climb: much higher than
# TAKEOFF_ALTITUDE_M, to leave plenty of climb time for GPS fusion to start and settle before the
# climb -- the "waypoint" a local takeoff's completion check looks at -- finishes.
MIDCLIMB_TAKEOFF_ALTITUDE_M = 40.0


def _send_local_takeoff_command(tester: AutopilotTesterFlow) -> None:
    """Send the local-frame takeoff command before arming: makes Takeoff (local position only)
    the mode arming is checked against, instead of whatever needs a global position by default.
    z is NED (down-positive), so the target altitude is negative."""
    tester.send_command_int(
        MAV_CMD_NAV_TAKEOFF, frame=MAV_FRAME_LOCAL_NED,
        x=round(LOITER_NORTH_M * LOCAL_FRAME_INT_SCALE),
        y=round(LOITER_EAST_M * LOCAL_FRAME_INT_SCALE),
        z=-TAKEOFF_ALTITUDE_M)


@pytest.mark.takeoff_local
def test_takeoff_local_offset_ned_frame_rejected(tester_flow: AutopilotTesterFlow) -> None:
    """ArduPilot sends a local-frame takeoff as MAV_FRAME_LOCAL_OFFSET_NED, not
    MAV_FRAME_LOCAL_NED. PX4 must reject it with MAV_RESULT_UNSUPPORTED_MAV_FRAME instead of
    letting it through: Commander/Navigator only recognise MAV_FRAME_LOCAL_NED as a local-frame
    takeoff, so x/y -- already scaled as local meters by mavlink_receiver.cpp -- would otherwise
    reach them as if they were global lat/lon degrees."""
    tester = tester_flow

    tester.send_command_int(
        MAV_CMD_NAV_TAKEOFF, frame=MAV_FRAME_LOCAL_OFFSET_NED,
        x=round(LOITER_NORTH_M * LOCAL_FRAME_INT_SCALE),
        y=round(LOITER_EAST_M * LOCAL_FRAME_INT_SCALE),
        z=-TAKEOFF_ALTITUDE_M,
        expected_result=MAV_RESULT_UNSUPPORTED_MAV_FRAME)


@pytest.mark.takeoff_local
def test_takeoff_local_reaches_altitude_then_lands(tester_flow: AutopilotTesterFlow) -> None:
    tester = tester_flow

    _send_local_takeoff_command(tester)
    tester.wait_until_armable()
    tester.arm()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 20)

    # A local-frame takeoff cannot be followed by Hold on this vehicle (it needs a global
    # position it never has), so it must stay in Takeoff instead of being forced into a Hold it
    # cannot run, or falling back to some other mode.
    tester.sleep_for(MODE_SETTLE_S)
    tester.wait_for_flight_mode(FlightMode.TAKEOFF, 1)

    tester.land()
    tester.wait_until_disarmed()


@pytest.mark.takeoff_local
def test_takeoff_local_home_from_raw_gps_without_fusion(tester_flow: AutopilotTesterFlow) -> None:
    """This airframe (sihsim_quadx_flow) still runs the simulated GPS (SENS_EN_GPSSIM 1); only
    EKF2_SENS_EN's GNSS bits are cleared, so EKF2 never fuses it into vehicle_global_position.
    Commander's home position is set independently of that: HomePosition falls back to the raw
    GPS fix whenever the fused global position is unavailable, gated by EKF2_GPS_CTRL (untouched
    here, so still enabled by default). Home must therefore end up valid while the fused global
    position stays invalid for the whole flight -- and the local-frame takeoff dispatch, which
    keys off global_position_invalid alone (Commander.cpp, navigator_main.cpp), must not be
    affected by home being valid."""
    tester = tester_flow

    _send_local_takeoff_command(tester)
    tester.wait_until_armable()

    # Home only ever latches for the first time while disarmed and landed (HomePosition's
    # check_if_changed gate), so by now it must already be valid -- not because of anything that
    # happens after arming.
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
    """Same starting point as test_takeoff_local_home_from_raw_gps_without_fusion: home is
    already valid (from the raw GPS fix, latched before arming) while the fused global position
    is not, for the whole climb. GPS fusion is then switched on in flight with
    MAV_CMD_ESTIMATOR_SENSOR_ENABLE, same as test_takeoff_local_gps_enabled_after_climb.

    The point is to isolate home validity as the one thing that differs from that other test:
    home was already valid here before the fused global position ever came up, instead of never
    having been valid at all. It must stay exactly as it was -- fusion starting must not move or
    invalidate a home that is already set -- and the vehicle must stay in Takeoff regardless, same
    as that other test: Commander's attempt to leave Takeoff for Hold already ran and failed once,
    right when the climb finished, and does not get a second attempt just because a global
    position (or, here, an already-valid home) is now in the picture. Fusion starting must also
    not physically move the vehicle: EKF2 resets its horizontal position/velocity to the GPS on
    the first fused sample, and that reset must be absorbed as a shift of the position estimate
    alone (via its reset counters), not felt by the position controller as a real tracking error
    to correct for."""
    tester = tester_flow

    _send_local_takeoff_command(tester)
    tester.wait_until_armable()

    # Home only ever latches for the first time while disarmed and landed (HomePosition's
    # check_if_changed gate), so by now it must already be valid -- not because of anything that
    # happens after arming.
    tester.wait_until_home_position_valid(10)
    tester.check_global_position_invalid()

    tester.arm()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 20)

    # Switch GPS fusion on in flight (EKF2_SENS_EN would only apply once disarmed).
    tester.send_command_long(
        MAV_CMD_ESTIMATOR_SENSOR_ENABLE, param1=FUSION_SOURCE_GPS, param2=0, param3=1)
    tester.wait_until_global_position_valid(15, max_speed_m_s=MAX_SPEED_DURING_FUSION_START_M_S)

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
    """With SENS_EN_GPSSIM 0 (set by this test's SITL config, overriding the airframe's default
    of 1) nothing ever publishes a GPS fix at all, the genuinely indoor/no-receiver case: unlike
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


@pytest.mark.takeoff_local_gps_midflight
def test_takeoff_local_gps_enabled_after_climb(tester_flow: AutopilotTesterFlow) -> None:
    """GPS starts out without a fix (SIM_GPS_USED 0, set by this test's SITL config), so the
    local-frame takeoff climbs exactly like test_takeoff_local_no_gps_sim. Once the takeoff
    altitude (its "waypoint") is reached, a fix is restored (SIM_GPS_USED back up) and then GPS
    fusion is switched on in flight with MAV_CMD_ESTIMATOR_SENSOR_ENABLE -- the command Commander
    itself would need for an in-flight sensor change, since EKF2_SENS_EN only takes effect while
    disarmed.

    The fused global position must then come up within a few seconds (EKF2 publishes it on its
    own, independently of Commander). Home position must not: HomePosition only re-evaluates a
    first-time home while disarmed and landed (Commander.cpp's
    "!isArmed() && landed && !_mission_in_progress" gate), so a GPS fix that only appears after
    arming must never retroactively move home.

    Commander's attempt to leave Takeoff for Hold is made once, right when the takeoff mission
    item finishes, and does not retry: by design, not as a limitation, a global position that
    only shows up afterwards must not belatedly pull the vehicle into Hold, so it stays in
    Takeoff for the rest of the flight regardless of what the GPS does later. Fusion starting
    must also not physically move the vehicle: EKF2's reset of its horizontal position/velocity
    to the GPS on the first fused sample must be absorbed as a shift of the estimate alone (via
    its reset counters), never felt by the position controller as a real tracking error to
    correct for.
    """
    tester = tester_flow

    _send_local_takeoff_command(tester)
    tester.wait_until_armable()
    tester.arm()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 20)

    tester.check_global_position_invalid()
    tester.check_home_position_invalid()

    # GPS now gets a fix, but home must stay unset: it is only latched while disarmed and landed.
    tester.set_param_int("SIM_GPS_USED", 10)
    tester.sleep_for(5)
    tester.check_home_position_invalid()
    tester.check_global_position_invalid()

    # Switch GPS fusion on in flight (EKF2_SENS_EN would only apply once disarmed).
    tester.send_command_long(
        MAV_CMD_ESTIMATOR_SENSOR_ENABLE, param1=FUSION_SOURCE_GPS, param2=0, param3=1)
    tester.wait_until_global_position_valid(15, max_speed_m_s=MAX_SPEED_DURING_FUSION_START_M_S)
    tester.check_home_position_invalid()

    # No retry: Commander's one-shot attempt to leave Takeoff for Hold already ran (and failed)
    # before fusion started, so the vehicle must stay in Takeoff for the rest of the flight.
    tester.sleep_for(MODE_SETTLE_S)
    tester.wait_for_flight_mode(FlightMode.TAKEOFF, 1)
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 5)

    tester.land()
    tester.wait_until_disarmed()


@pytest.mark.takeoff_local_gps_already_fused
def test_takeoff_local_reaches_hold_when_gps_already_fused(
        tester_flow: AutopilotTesterFlow) -> None:
    """EKF2_SENS_EN is restored to its default here (this test's SITL config overrides the
    airframe's "disable gps fusion" bitclear back to its default of 8191), so GPS fusion -- and
    the fused global position with it -- is already active well before the local-frame takeoff is
    even sent. A local-frame takeoff still makes sense to request on a vehicle like this
    (MAV_FRAME_LOCAL_NED is an explicit request, independent of whether a global position happens
    to be available) and it still climbs exactly the same way as every other test here.

    The difference shows up the moment the climb finishes: Commander's one-shot attempt to leave
    Takeoff for Hold (see the other tests in this file, which all provoke it to fail) finds
    canRun(AUTO_LOITER) already true, since global_position_invalid was never true to begin with,
    so it succeeds immediately instead of leaving the vehicle stuck. Because the switch happens
    essentially as soon as the climb completes -- before Takeoff's own internal quasi-loiter has
    had any time to carry the vehicle towards the commanded loiter point -- Hold's freshly
    computed global-frame target (Loiter::on_activation()) should coincide almost exactly with
    wherever the vehicle already is. This is the ordinary, fully-working case the rest of this
    file's deliberately-provoked failures and large drifts must not be mistaken for a regression
    of."""
    tester = tester_flow

    _send_local_takeoff_command(tester)
    tester.wait_until_armable()
    tester.arm()
    tester.wait_until_global_position_valid(10)
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 20)

    north_m, east_m, _down_m = tester.get_current_position_ned()

    # Unlike every other test in this file, Hold must actually be reached, and quickly: Commander's
    # one-shot attempt succeeds on the first try since global_position_invalid was never true.
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
    """Same starting point as test_takeoff_local_home_from_raw_gps_without_fusion (home valid
    from the raw fix, fused global position not -- this airframe's "disable gps fusion" bitclear
    is untouched), but here GPS fusion is switched on with MAV_CMD_ESTIMATOR_SENSOR_ENABLE
    partway *through* the climb, well before it finishes -- not after, like
    test_takeoff_local_fuses_gps_after_climb_with_home_already_valid, and not before the takeoff
    was ever sent, like test_takeoff_local_reaches_hold_when_gps_already_fused. The takeoff
    climbs to MIDCLIMB_TAKEOFF_ALTITUDE_M, much higher than every other test here, specifically to
    leave enough climb time for the fix to settle well before the "waypoint" (the target
    altitude, which is all a local takeoff's completion check looks at) is reached.

    By the time the climb does finish, canRun(AUTO_LOITER) has already been true for a while, so
    Commander's one-shot attempt to leave Takeoff for Hold succeeds on its only try -- the same
    outcome as test_takeoff_local_reaches_hold_when_gps_already_fused, but this time the global
    position only became available mid-flight rather than before the takeoff was ever sent.

    The takeoff requests no horizontal target at all (north = east = 0, straight up), unlike
    every other test in this file: that leaves nothing for Takeoff's own quasi-loiter to carry
    the vehicle towards even if it got the chance to run, so any drift measured switching into
    Hold can only come from the frame switch itself (EKF2's reset absorbed via its reset
    counters, Hold's fresh global-frame target from Loiter::on_activation()) -- not from a
    confounding horizontal transit towards a commanded loiter point, unlike the diagonal loiter
    points used elsewhere in this file."""
    tester = tester_flow

    tester.send_command_int(
        MAV_CMD_NAV_TAKEOFF, frame=MAV_FRAME_LOCAL_NED,
        x=0, y=0, z=-MIDCLIMB_TAKEOFF_ALTITUDE_M)
    tester.wait_until_armable()

    # Home only ever latches for the first time while disarmed and landed (HomePosition's
    # check_if_changed gate), so by now it must already be valid -- not because of anything that
    # happens after arming.
    tester.wait_until_home_position_valid(10)
    tester.check_global_position_invalid()

    tester.arm()

    # Partway through the climb: neither right at liftoff nor anywhere near the top.
    tester.sleep_for(5)

    # Switch GPS fusion on in flight (EKF2_SENS_EN would only apply once disarmed).
    tester.send_command_long(
        MAV_CMD_ESTIMATOR_SENSOR_ENABLE, param1=FUSION_SOURCE_GPS, param2=0, param3=1)
    tester.wait_until_global_position_valid(15, max_speed_m_s=MAX_SPEED_DURING_FUSION_START_M_S)

    # Confirm this really happened mid-climb, not coincidentally right at the top. This doubles as
    # the "before" snapshot for the drift check below: with north = east = 0 the whole flight,
    # horizontal position should not move no matter when it is sampled, climbing or holding.
    north_m, east_m, mid_down_m = tester.get_current_position_ned()
    mid_climb_altitude_m = -mid_down_m
    assert mid_climb_altitude_m < MIDCLIMB_TAKEOFF_ALTITUDE_M - 10.0, \
        "Already at {:.1f} m of {} m when the global position became valid, too close to the " \
        "top to show this happening mid-climb".format(
            mid_climb_altitude_m, MIDCLIMB_TAKEOFF_ALTITUDE_M)

    # Same outcome as test_takeoff_local_reaches_hold_when_gps_already_fused: Commander's one and
    # only attempt succeeds immediately, since canRun(AUTO_LOITER) has been true for a while. Hold
    # activates once Navigator's own (looser) altitude-acceptance check passes -- not necessarily
    # within this test's tighter wait_until_altitude tolerance, and Hold then holds at whatever
    # altitude that was rather than continuing on to the exact commanded one -- so this waits for
    # the mode switch itself, generously, rather than for an exact altitude match.
    tester.wait_for_flight_mode(FlightMode.HOLD, 90)

    new_north_m, new_east_m, _new_down_m = tester.get_current_position_ned()
    drift_m = math.hypot(new_north_m - north_m, new_east_m - east_m)
    assert drift_m < MAX_HOLD_HANDOVER_DRIFT_M, \
        "Horizontal drift {:.2f} m switching into Hold, expected < {} m".format(
            drift_m, MAX_HOLD_HANDOVER_DRIFT_M)

    tester.land()
    tester.wait_until_disarmed()
