import statistics
from typing import List

import pytest

from autopilot_tester import AutopilotTester

IMU_WINDOW_S = 4.0
# SCALED_IMU carries the IMU after integration, which averages the noise down: in a hover the
# standard deviation on the right and down axes is about 0.07 m/s^2 with SIH_IMU_VIB_EN disabled
# and about 1.2 and 1.0 m/s^2 with it enabled.
ACCEL_NOISE_THRESHOLD_M_S2 = 0.4
RIGHT, DOWN = 1, 2


def accel_std(tester: AutopilotTester) -> List[float]:
    """Per-axis standard deviation of the accelerometer over IMU_WINDOW_S."""
    samples = tester.record_scaled_imu(IMU_WINDOW_S)
    axes = [(s.acceleration_frd.forward_m_s2, s.acceleration_frd.right_m_s2,
             s.acceleration_frd.down_m_s2) for s in samples]
    return [statistics.pstdev(axis) for axis in zip(*axes)]


@pytest.mark.sih_imu_noise
def test_in_flight_imu_noise_follows_parameter(tester: AutopilotTester) -> None:
    """SIH_IMU_VIB_EN is disabled by default and, when enabled, raises the IMU noise in flight."""
    tester.wait_until_ready()
    tester.arm_and_take_off(5.0)
    # Let the takeoff overshoot damp out, so that only the sensor noise is left in the hover.
    tester.sleep_for(3)

    quiet = accel_std(tester)
    tester.set_param_int("SIH_IMU_VIB_EN", 1)
    tester.sleep_for(1)
    noisy = accel_std(tester)

    for axis in (RIGHT, DOWN):
        assert quiet[axis] < ACCEL_NOISE_THRESHOLD_M_S2, \
            "Accelerometer noise {:.2f} m/s^2 with SIH_IMU_VIB_EN disabled".format(quiet[axis])
        assert noisy[axis] > ACCEL_NOISE_THRESHOLD_M_S2, \
            "Accelerometer noise {:.2f} m/s^2 with SIH_IMU_VIB_EN enabled".format(noisy[axis])
