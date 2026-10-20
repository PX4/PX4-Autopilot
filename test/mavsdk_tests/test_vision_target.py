"""Precision landing on a static pad with the vision target estimator."""

import pytest

from autopilot_tester_vision_target import AutopilotTesterVisionTarget, Scenario


@pytest.mark.vision_target
@pytest.mark.parametrize("scenario", list(Scenario), ids=[s.value for s in Scenario])
def test_precision_land(tester_vision_target: AutopilotTesterVisionTarget,
                        scenario: Scenario) -> None:
    """Scenarios:

    - vision: TARGET_RELATIVE only.
    - gnss: TARGET_ABSOLUTE and vehicle GNSS velocity.
    - vision_and_gnss: both target messages.
    - mission: landing from a mission whose land position is off, corrected by vision.
    - gnss_jump: the target GNSS position jumps and has to be rejected.
    - vision_loss: the GNSS bias learned from vision is retained after vision is lost.
    """
    tester = tester_vision_target
    tester.wait_until_ready()

    # The pad stays fixed on the ground, well outside the acceptance radius of a normal
    # landing at takeoff. Only the simulated target sensor tells PX4 where it is.
    takeoff_altitude_m = 12.0
    tester.set_landing_target(5.0, 3.0)
    tester.set_takeoff_altitude(takeoff_altitude_m)
    tester.arm()
    tester.takeoff()
    tester.wait_until_altitude(takeoff_altitude_m, 30)

    tester.precision_land(scenario, 90)
    tester.check_landed_at_target(0.5)
