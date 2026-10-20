import pytest

from autopilot_tester import plan_file
from autopilot_tester_rtl import AutopilotTesterRtl

MISSION = plan_file("vtol_mission.plan")
MISSION_WITH_LAND_START = plan_file("vtol_mission_with_land_start.plan")
MISSION_WITHOUT_LANDING = plan_file("vtol_mission_without_landing.plan")

# The C++ tests waited 60 s of host time for the approaches, which is several times that in sim
# time at the speed factors used in CI.
APPROACH_TIMEOUT_S = 300


def prepare(tester: AutopilotTesterRtl, plan: str) -> None:
    tester.wait_until_ready()
    tester.store_home()
    tester.load_qgc_mission_raw_and_move_here(plan)


@pytest.mark.vtol
def test_return_direct_home(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    prepare(tester, MISSION_WITH_LAND_START)
    # Fly directly to the home position.
    tester.set_rtl_type(0)
    tester.set_rtl_appr_force(0)
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(2)
    tester.wait_until_disarmed(150)
    tester.check_home_within(5.0)


@pytest.mark.vtol
def test_return_direct_mission_land(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    prepare(tester, MISSION_WITH_LAND_START)
    # Do not allow home.
    tester.set_rtl_type(1)
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(2)
    tester.wait_until_disarmed(150)
    tester.check_mission_land_within(5.0)


@pytest.mark.vtol
def test_return_direct_mission_land_preserves_multicopter_mode(
        tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    prepare(tester, MISSION_WITH_LAND_START)
    tester.set_rtl_type(1)
    tester.arm()
    # Item 6 is the DO_LAND_START.
    tester.start_mission_raw_and_wait_for_sequence(6)
    tester.transition_to_multicopter()
    tester.wait_until_multicopter(60)
    tester.execute_rtl()
    tester.wait_until_disarmed_while_in_multicopter_mode(180)
    tester.check_mission_land_within(5.0)


@pytest.mark.vtol
def test_return_with_mission_landing(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    tester.wait_until_ready()
    tester.load_qgc_mission_raw_and_move_here(MISSION)
    # The vehicle should follow the mission and use the mission landing.
    tester.set_rtl_type(2)
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(2)
    tester.check_tracks_mission_raw(40.0)
    tester.wait_until_disarmed(150)


@pytest.mark.vtol
def test_return_with_reverse_mission(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    tester.wait_until_ready()
    tester.set_takeoff_land_requirements(0)
    tester.load_qgc_mission_raw_and_move_here(MISSION_WITHOUT_LANDING)
    # The vehicle should follow the mission in reverse and land at the home position.
    # TODO: check the track with check_tracks_mission_raw(35.0, reverse=True) again.
    tester.set_rtl_type(2)
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(6)
    tester.wait_until_disarmed(150)


@pytest.mark.vtol
def test_return_with_mission_landing_preserves_multicopter_mode(
        tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    prepare(tester, MISSION)
    tester.set_rtl_type(2)
    tester.arm()
    # In fixed-wing by then, half way round the mission.
    tester.start_mission_raw_and_wait_for_sequence(4)
    tester.transition_to_multicopter()
    tester.wait_until_multicopter(60)
    tester.execute_rtl()
    # The rest of the mission and its landing are flown as a multicopter.
    tester.wait_until_disarmed_while_in_multicopter_mode(300)
    tester.check_mission_land_within(5.0)


@pytest.mark.vtol
def test_return_with_reverse_mission_preserves_multicopter_mode(
        tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    tester.wait_until_ready()
    tester.store_home()
    tester.set_takeoff_land_requirements(0)
    tester.load_qgc_mission_raw_and_move_here(MISSION_WITHOUT_LANDING)
    tester.set_rtl_type(2)
    tester.arm()
    tester.start_mission_raw_and_wait_for_sequence(4)
    tester.transition_to_multicopter()
    tester.wait_until_multicopter(60)
    tester.execute_rtl()
    # The mission is flown back in reverse as a multicopter down to the home position.
    tester.wait_until_disarmed_while_in_multicopter_mode(300)
    tester.check_home_within(5.0)


@pytest.mark.vtol
@pytest.mark.parametrize("approach_force", [
    pytest.param(0, id="not_forced"),
    pytest.param(1, id="forced"),
])
def test_return_direct_home_without_approaches(tester_rtl: AutopilotTesterRtl,
                                               approach_force: int) -> None:
    tester = tester_rtl
    prepare(tester, MISSION_WITH_LAND_START)
    tester.set_rtl_type(0)
    tester.set_rtl_appr_force(approach_force)
    tester.add_home_to_rally_point()
    tester.upload_rally_points()
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(4)
    tester.wait_until_disarmed(150)
    if approach_force:
        # Home has no approaches, so it can't be used and the mission landing is.
        tester.check_mission_land_within(5.0)
    else:
        tester.check_home_within(5.0)


@pytest.mark.vtol
def test_return_direct_home_with_approaches(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    prepare(tester, MISSION_WITH_LAND_START)
    tester.set_rtl_type(0)
    tester.add_home_with_approaches_to_rally_point()
    tester.upload_rally_points()
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(4)
    tester.check_rtl_approaches(5.0, APPROACH_TIMEOUT_S)
    tester.wait_until_disarmed(180)
    tester.check_home_within(5.0)


@pytest.mark.vtol
def test_return_direct_home_not_as_rally_point(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    prepare(tester, MISSION_WITH_LAND_START)
    tester.set_rtl_type(1)
    tester.add_home_with_approaches_to_rally_point()
    tester.upload_rally_points()
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(2)
    tester.wait_until_disarmed(150)
    tester.check_mission_land_within(5.0)


@pytest.mark.vtol
def test_return_direct_rally_without_approaches(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    prepare(tester, MISSION_WITH_LAND_START)
    # Do not allow the home position.
    tester.set_rtl_type(1)
    tester.set_rtl_appr_force(0)
    tester.add_local_rally_point(100.0, -200.0)
    tester.upload_rally_points()
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(3)
    tester.wait_until_disarmed(150)
    tester.check_rally_point_within(5.0)
    tester.check_home_not_within(20.0)


@pytest.mark.vtol
def test_return_direct_rally_without_approaches_forced(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    prepare(tester, MISSION_WITH_LAND_START)
    tester.set_rtl_type(1)
    tester.set_rtl_appr_force(1)
    # The rally point has no approaches, so it can't be used and the mission landing is.
    tester.add_local_rally_point(100.0, -2000.0)
    tester.upload_rally_points()
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(3)
    tester.wait_until_disarmed(150)
    tester.check_mission_land_within(5.0)


@pytest.mark.vtol
def test_return_direct_rally_with_approaches(tester_rtl: AutopilotTesterRtl) -> None:
    tester = tester_rtl
    prepare(tester, MISSION_WITH_LAND_START)
    tester.set_rtl_type(1)
    tester.set_rtl_appr_force(0)
    tester.add_local_rally_with_approaches_point(100.0, -200.0)
    tester.upload_rally_points()
    tester.arm()
    tester.execute_rtl_when_reaching_mission_sequence(3)
    tester.check_rtl_approaches(5.0, APPROACH_TIMEOUT_S)
    tester.wait_until_disarmed(150)
    tester.check_rally_point_within(5.0)
    tester.check_home_not_within(20.0)
