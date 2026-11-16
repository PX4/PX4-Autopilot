import pytest
from mavsdk.plugins.offboard import PositionNedYaw  # type: ignore

from autopilot_tester import AutopilotTester, RcLossException

ACCEPTANCE_RADIUS_M = 0.3

TAKEOFF_POSITION = PositionNedYaw(north_m=0.0, east_m=0.0, down_m=-2.0, yaw_deg=0.0)


@pytest.mark.multicopter
@pytest.mark.offboard
def test_offboard_takeoff_and_land(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    tester.store_home()
    tester.set_rc_loss_exception(RcLossException.OFFBOARD)
    tester.arm()
    tester.offboard_goto(TAKEOFF_POSITION, ACCEPTANCE_RADIUS_M, 90)
    tester.offboard_land()
    tester.wait_until_disarmed(120)
    tester.check_home_within(2.0)


@pytest.mark.multicopter
@pytest.mark.offboard
def test_offboard_position_control(tester: AutopilotTester) -> None:
    setpoints = [
        TAKEOFF_POSITION,
        PositionNedYaw(north_m=0.0, east_m=5.0, down_m=-2.0, yaw_deg=180.0),
        PositionNedYaw(north_m=5.0, east_m=5.0, down_m=-4.0, yaw_deg=180.0),
        PositionNedYaw(north_m=5.0, east_m=0.0, down_m=-4.0, yaw_deg=90.0),
        TAKEOFF_POSITION,
    ]

    tester.wait_until_ready()
    tester.store_home()
    tester.set_rc_loss_exception(RcLossException.OFFBOARD)
    tester.arm()
    for setpoint in setpoints:
        tester.offboard_goto(setpoint, ACCEPTANCE_RADIUS_M, 10)
    tester.offboard_land()
    tester.wait_until_disarmed(120)
    tester.check_home_within(2.0)


@pytest.mark.offboard_attitude
def test_offboard_attitude_control(tester: AutopilotTester) -> None:
    tester.set_rc_loss_exception(RcLossException.OFFBOARD)
    tester.fly_forward_in_offboard_attitude()
    tester.wait_until_disarmed(120)
