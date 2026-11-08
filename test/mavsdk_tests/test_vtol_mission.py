import pytest

from autopilot_tester import AutopilotTester, plan_file


@pytest.mark.vtol
def test_vtol_mission(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    tester.load_qgc_mission_raw_and_move_here(plan_file("vtol_mission.plan"))
    tester.arm()
    tester.execute_mission_raw()
    tester.wait_until_disarmed()
