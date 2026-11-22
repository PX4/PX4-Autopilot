import pytest

from autopilot_tester import AutopilotTester, MissionOptions


@pytest.mark.multicopter
@pytest.mark.vtol
def test_takeoff_and_land(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.land()
    tester.wait_until_disarmed(180)


@pytest.mark.multicopter
def test_square_mission_with_return(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    tester.prepare_square_mission(MissionOptions(rtl_at_end=True))
    tester.arm()
    tester.execute_mission()
    tester.wait_until_disarmed(180)


@pytest.mark.multicopter
def test_square_mission_with_manual_return(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    tester.prepare_square_mission(MissionOptions(rtl_at_end=False))
    tester.check_tracks_mission(5.0)
    tester.arm()
    tester.execute_mission()
    tester.wait_until_hovering()
    tester.execute_rtl()
    tester.wait_until_disarmed(180)


@pytest.mark.multicopter
def test_straight_mission(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    tester.prepare_straight_mission(MissionOptions(rtl_at_end=False, fly_through=True))
    tester.check_mission_item_speed_above(2, 4.0)
    tester.check_tracks_mission(5.0)
    tester.arm()
    tester.execute_mission()
    tester.wait_until_hovering()
    tester.execute_rtl()
    tester.wait_until_disarmed(180)
