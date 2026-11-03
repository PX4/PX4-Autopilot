import pytest
from mavsdk.plugins.offboard import PositionNedYaw  # type: ignore

from autopilot_tester_rtl import AutopilotTesterRtl


# TODO: Add test cases for these scenarios:
# - Return type 6 home within reach with no rally points
# - Return type 6 home within reach with rally points and home is the closest
# - Return type 6 home within reach with rally points and home is not the closest
# - Return type 6 home out of reach with rally points and at least one rally point within reach

@pytest.mark.multicopter
def test_rtl_type_6_no_rally_points(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    tester.wait_until_ready()
    tester.store_home()
    tester.set_rtl_type(6)
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.execute_rtl()
    tester.wait_until_disarmed(90)
    tester.check_home_within(5.0)


@pytest.mark.multicopter
def test_rtl_type_6_home_is_closest(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    tester.wait_until_ready()
    tester.store_home()
    tester.set_rtl_type(6)
    # The rally point is farther away than home, and the vehicle is above home after takeoff,
    # so home wins on distance.
    tester.add_local_rally_point(100.0, -100.0)
    tester.upload_rally_points()
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.execute_rtl()
    tester.wait_until_disarmed(90)
    tester.check_home_within(5.0)


@pytest.mark.multicopter
def test_rtl_type_6_home_is_not_closest(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    tester.wait_until_ready()
    tester.store_home()
    tester.set_rtl_type(6)
    # The rally point is between the vehicle and home, so it wins on distance.
    tester.add_local_rally_point(300.0, 0.0)
    tester.upload_rally_points()
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    # Distance to home is 400 m, distance to the rally point is 100 m.
    tester.offboard_goto(PositionNedYaw(north_m=400.0, east_m=0.0, down_m=-20.0, yaw_deg=0.0),
                         5.0, 60)
    tester.execute_rtl()
    tester.wait_until_disarmed(150)
    tester.check_rally_point_within(5.0)
    tester.check_home_not_within(20.0)
