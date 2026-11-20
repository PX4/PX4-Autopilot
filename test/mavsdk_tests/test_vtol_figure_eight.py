import math

import pytest

from autopilot_tester_loiter import AutopilotTesterLoiter

TAKEOFF_ALTITUDE_M = 20.0


@pytest.mark.vtol
@pytest.mark.parametrize("major_axis_m, orientation_rad", [
    pytest.param(150.0, 0.0, id="clockwise"),
    pytest.param(-150.0, math.radians(30.0), id="counterclockwise"),
])
def test_figure_eight(tester_loiter: AutopilotTesterLoiter, major_axis_m: float,
                      orientation_rad: float) -> None:
    tester = tester_loiter
    center = (200.0, 0.0)
    minor_axis_m = 50.0

    tester.wait_until_ready()
    tester.store_home()
    tester.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)
    tester.sleep_for(3)
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 30)
    tester.transition_to_fixedwing()
    tester.wait_until_fixedwing(5)
    tester.sleep_for(1)

    tester.command_figure_eight(center, major_axis_m, minor_axis_m, 20.0, orientation_rad)
    # The C++ test waited 120 s of host time.
    tester.check_tracks_figure_eight(center, major_axis_m, minor_axis_m, orientation_rad,
                                     corridor_radius_m=10.0, timeout_s=300)
    # TODO: check FIGURE_EIGHT_EXECUTION_STATUS, which MAVSDK can now receive with MavlinkDirect.
