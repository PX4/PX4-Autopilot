import pytest

from autopilot_tester import AutopilotTester


@pytest.mark.multicopter
@pytest.mark.vtol
def test_fly_forward_in_position_control(tester: AutopilotTester) -> None:
    tester.fly_forward_in_posctl()
    tester.wait_until_disarmed(180)
    tester.check_home_not_within(5.0)


@pytest.mark.multicopter
@pytest.mark.vtol
def test_fly_forward_in_altitude_control(tester: AutopilotTester) -> None:
    tester.fly_forward_in_altctl()
    tester.wait_until_disarmed(180)
    tester.check_home_not_within(5.0)
