import pytest

from autopilot_tester import AutopilotTester, plan_file


@pytest.mark.vtol_wind
def test_vtol_mission_with_wind_change(tester: AutopilotTester) -> None:
    """Needs the ramped_up_wind world, see configs/gazebo-classic-sitl.json."""
    tester.wait_until_ready()

    # Forward thrust in land is more robust in wind (less lift).
    tester.set_param_int("VT_FWD_THRUST_EN", 1)
    tester.enable_fixedwing_metrics()

    tester.load_qgc_mission_raw_and_move_here(plan_file("vtol_mission_straight_south.plan"))
    tester.arm()
    tester.execute_mission_raw()
    tester.wait_until_disarmed()

    # Checking once after landing is enough, as the invalidation is permanent.
    tester.check_airspeed_is_valid()
