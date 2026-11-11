import math

import pytest

from autopilot_tester_moving_platform import AutopilotTesterMovingPlatform
from takeoff_checks import check_takeoff

TAKEOFF_ALTITUDE_M = 5.0


@pytest.mark.moving_platform
def test_takeoff_from_moving_platform(
        tester_moving_platform: AutopilotTesterMovingPlatform) -> None:
    """Take off from the SIH platform moving with SIH_PLAT_VN through air at rest."""
    tester = tester_moving_platform
    platform_ne = tester.platform_velocity_ne()
    assert math.hypot(*platform_ne) > 0.5, "Configure a moving platform with SIH_PLAT_VN"

    tester.wait_until_ready()
    tester.wait_until_estimate_tracks_platform(tolerance_m_s=0.3, timeout_s=10)

    tester.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)
    tester.arm()
    tester.start_recording_takeoff()
    tester.takeoff()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 15)
    recording = tester.stop_recording_takeoff(platform_ne)
    tester.wait_until_speed_lower_than(0.5, 15)

    # No landing: the land detector checks the velocity over ground and never detects a landing
    # on the moving platform.

    failures = check_takeoff(recording, platform_ne, tester.drag_per_kg())
    assert not failures, "; ".join(failures)
