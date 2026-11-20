import pytest

from autopilot_tester import AutopilotTester


@pytest.mark.sih_xvert_transition
def test_sih_xvert_front_transition(tester: AutopilotTester) -> None:
    takeoff_altitude_m = 30.0

    tester.wait_until_ready()
    tester.set_takeoff_altitude(takeoff_altitude_m)
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(takeoff_altitude_m, 30)
    tester.transition_to_fixedwing()
    tester.wait_until_fixedwing(15)
    tester.sleep_for(10)
    # Still in fixed-wing after 10 s.
    tester.wait_until_fixedwing(1)
