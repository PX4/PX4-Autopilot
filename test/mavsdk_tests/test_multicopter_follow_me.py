import pytest

from autopilot_tester_follow_me import AutopilotTesterFollowMe

TAKEOFF_ALTITUDE_M = 10.0


def take_off(tester: AutopilotTesterFollowMe) -> None:
    tester.wait_until_ready()
    tester.store_home()
    tester.arm()
    tester.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)
    tester.takeoff()
    tester.wait_until_hovering()


@pytest.mark.multicopter
@pytest.mark.parametrize("stream_velocity", [
    pytest.param(False, id="position"),
    pytest.param(True, id="position_and_velocity"),
])
def test_follow_target_streaming(tester_follow_me: AutopilotTesterFollowMe,
                                 stream_velocity: bool) -> None:
    tester = tester_follow_me
    take_off(tester)
    tester.straight_line_test(stream_velocity)
    tester.land()
    tester.wait_until_disarmed(300)


@pytest.mark.multicopter
def test_follow_target_velocity_streaming_only(tester_follow_me: AutopilotTesterFollowMe) -> None:
    # Follow-me needs at least one position to integrate the velocity, so the drone must not move.
    tester = tester_follow_me
    take_off(tester)
    tester.stream_velocity_only()
    tester.land()
    tester.wait_until_disarmed(300)
    tester.check_home_within(1.0)


@pytest.mark.multicopter
def test_follow_target_rc_adjustment(tester_follow_me: AutopilotTesterFollowMe) -> None:
    tester = tester_follow_me
    take_off(tester)
    tester.rc_adjustment_test()
    tester.land()
    tester.wait_until_disarmed(300)
