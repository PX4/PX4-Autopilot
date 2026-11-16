import math

import pytest

from autopilot_tester import AutopilotTester


def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@pytest.mark.sih_home_yaw
def test_home_yaw_captured_when_local_position_invalid(tester: AutopilotTester) -> None:
    """Reproduce the stale home yaw bug: when home is first set from a raw GPS fix while the
    local position estimate is invalid, home yaw must still end up at the true ground heading."""

    # No GPS fix yet: wait for the magnetometer heading to align and settle, so that no later
    # heading reset can refresh home yaw and mask the bug.
    tester.sleep_for(10)
    ground_yaw_deg = float(tester.get_attitude_euler().yaw_deg)

    # Degraded GPS: good enough for the home position, too few satellites for EKF fusion, so the
    # local position stays invalid. This is the exact condition under which the buggy code left
    # home yaw at 0.
    tester.set_param_int("SIM_GPS_USED", 5)
    tester.sleep_for(6)

    q = tester.get_home_position()["q"]
    home_yaw_deg = math.degrees(yaw_from_quaternion(q[0], q[1], q[2], q[3]))

    diff_deg = abs(ground_yaw_deg - home_yaw_deg)
    if diff_deg > 180.0:
        diff_deg = 360.0 - diff_deg

    # With the fix home yaw tracks the true ground heading; with the bug it stays at 0 (north).
    assert diff_deg < 20.0, "Home yaw {:.1f} deg, ground yaw {:.1f} deg".format(
        home_yaw_deg, ground_yaw_deg)
