"""Local-frame takeoff on a GPS-less multicopter (sihsim_quadx_flow), commanded with
MAV_CMD_NAV_TAKEOFF in MAV_FRAME_LOCAL_NED, with no global position ever available.

PX4 no longer has a dedicated local-frame takeoff command: a local-frame takeoff is requested by
sending the ordinary MAV_CMD_NAV_TAKEOFF as a COMMAND_INT with frame = MAV_FRAME_LOCAL_NED,
rather than by command id. VEHICLE_CMD_NAV_TAKEOFF_LOCAL still exists as a vehicle_command/MAVLink
enum value, but PX4 no longer dispatches on it.

Unlike test_multicopter_optical_flow.py, nothing here ever gives the estimator an origin (no
SET_GPS_GLOBAL_ORIGIN), so the vehicle never gets a global position, not even a coarse one.
Arming on this vehicle normally needs one (whatever mode is selected by default requires it), so
the takeoff command is sent *before* arming: a local-frame takeoff only needs a local position,
and sending it first makes Takeoff (not the default mode) the one arming is checked against.

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

import pytest
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester_flow import AutopilotTesterFlow

MAV_CMD_NAV_TAKEOFF = 22
MAV_FRAME_LOCAL_NED = 1

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


@pytest.mark.takeoff_local
def test_takeoff_local_reaches_altitude_then_lands(tester_flow: AutopilotTesterFlow) -> None:
    tester = tester_flow

    # Sent before arming: makes Takeoff (local position only) the mode arming is checked
    # against, instead of whatever needs a global position by default.
    # z is NED (down-positive), so the target altitude is negative.
    tester.send_command_int(
        MAV_CMD_NAV_TAKEOFF, frame=MAV_FRAME_LOCAL_NED,
        x=round(LOITER_NORTH_M * LOCAL_FRAME_INT_SCALE),
        y=round(LOITER_EAST_M * LOCAL_FRAME_INT_SCALE),
        z=-TAKEOFF_ALTITUDE_M)

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
