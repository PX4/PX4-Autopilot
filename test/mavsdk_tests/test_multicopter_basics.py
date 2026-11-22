import pytest

from autopilot_tester import AutopilotTester


@pytest.mark.multicopter
@pytest.mark.vtol
def test_takeoff_and_hold_position(tester: AutopilotTester) -> None:
    takeoff_altitude_m = 10.0
    # Takeoff overshoot plus the slow VTOL settle can drift the hold altitude by ~0.1 m; keep the
    # tolerance above that so a normal hover doesn't trip it.
    altitude_hold_tolerance_m = 0.2

    tester.wait_until_ready()

    tester.set_takeoff_altitude(takeoff_altitude_m)
    tester.store_home()
    # The sleep here is necessary for the takeoff altitude to be applied properly.
    tester.sleep_for(1)

    ground_altitude_m = -tester.get_current_position_ned()[2]

    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(ground_altitude_m + takeoff_altitude_m, 15, delta=0.1)

    # wait_until_altitude() returns on the first touch of the target band, while the vehicle is
    # still climbing. Let the takeoff overshoot damp out so the hold reference is captured at the
    # settled hover altitude.
    tester.sleep_for(5)

    tester.start_checking_altitude(altitude_hold_tolerance_m)
    tester.sleep_for(15)
