import pytest
from mavsdk.plugins.failure import FailureType, FailureUnit  # type: ignore

from autopilot_tester import AutopilotTester, plan_file


@pytest.mark.vtol_airspeed_fail
def test_vtol_loiter_with_airspeed_failure(tester: AutopilotTester) -> None:
    takeoff_altitude_m = 10.0

    tester.wait_until_ready()
    tester.enable_fixedwing_metrics()
    tester.set_takeoff_altitude(takeoff_altitude_m)
    tester.load_qgc_mission_raw_and_move_here(plan_file("vtol_mission.plan"))
    tester.arm()

    tester.takeoff()
    tester.wait_until_altitude(takeoff_altitude_m, 30)
    tester.transition_to_fixedwing()

    tester.sleep_for(10)
    tester.inject_failure(FailureUnit.SENSOR_AIRSPEED, FailureType.WRONG)
    tester.sleep_for(10)

    tester.check_airspeed_is_invalid()
