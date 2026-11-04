import pytest

from autopilot_tester import AutopilotTester, MissionOptions


@pytest.mark.multicopter
def test_external_set_current_onto_do_jump(tester: AutopilotTester) -> None:
    """Regression test for the DO_JUMP "idle setpoint" hang.

    An external MISSION_SET_CURRENT sets the navigator's current sequence verbatim, without jump
    resolution. If it points directly at a DO_JUMP item, the navigator used to load that jump as
    the current item, publish an invalid IDLE position setpoint, and hang there indefinitely
    (observed in a real flight). The fix resolves the jump when loading the current item.

    The mission contains a DO_JUMP (item 3, looping back to waypoint 1). Mid-mission, we send
    MISSION_SET_CURRENT pointing at it. The mission must still complete, land and disarm.
    """
    tester.wait_until_ready()

    jump_index = tester.prepare_multicopter_mission_with_do_jump(
        MissionOptions(leg_length_m=100.0, relative_altitude_m=15.0), jump_repeats=1)
    tester.arm()

    tester.start_mission_raw_and_wait_for_sequence(2)
    tester.send_set_current_mission_item(jump_index)

    # With the bug the vehicle would hang and never land.
    tester.wait_until_disarmed(240)
