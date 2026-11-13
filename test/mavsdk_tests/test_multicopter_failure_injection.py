import statistics
from typing import List

import pytest
from mavsdk.plugins.failure import FailureResult, FailureType, FailureUnit  # type: ignore
from mavsdk.plugins.offboard import PositionNedYaw  # type: ignore

from autopilot_tester import AutopilotTester

FLIGHT_ALTITUDE_M = 10.0
ALTITUDE_TOLERANCE_M = 1.0
HOVER_SPEED_TOLERANCE_M_S = 1.0
NUM_MOTORS = 6
MOTOR_INSTANCE = 1  # 1-based

IMU_INSTANCE = 1  # 1-based, SIH has a single IMU
IMU_FAILURE_DURATION_S = 20
IMU_WINDOW_S = 3


@pytest.mark.multicopter
def test_reject_mid_air_when_disabled(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    tester.set_param_int("SYS_FAILURE_EN", 0)
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.inject_failure(FailureUnit.SYSTEM_MOTOR, FailureType.OFF, 1,
                          expected_result=FailureResult.DISABLED)
    tester.execute_rtl()
    tester.wait_until_disarmed(180)


def take_off_and_hover_for_motor_failure(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    tester.set_param_int("SYS_FAILURE_EN", 1)
    # Airmode for control allocation with a failed motor.
    tester.set_param_int("MC_AIRMODE", 1)
    # Remove the failed motor from the allocation and stop the opposite one.
    tester.set_param_int("CA_FAILURE_MODE", 1)
    tester.set_takeoff_altitude(FLIGHT_ALTITUDE_M)
    tester.enable_actuator_output_status()
    # Necessary for the takeoff altitude to be applied properly.
    tester.sleep_for(1)

    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(FLIGHT_ALTITUDE_M, 30)
    tester.wait_until_speed_lower_than(HOVER_SPEED_TOLERANCE_M_S, 30)
    # Both waits pass while the vehicle is still slowing down at the top of the climb. Let it
    # settle, so the altitude reference for the failure is the hover altitude, not mid-climb.
    tester.sleep_for(5)


def fail_and_restore_motor(tester: AutopilotTester, failure_type: FailureType,
                           stopped_mask: int) -> None:
    # The vehicle must hold altitude through the failure and the recovery.
    tester.start_checking_altitude(ALTITUDE_TOLERANCE_M)

    tester.inject_failure(FailureUnit.SYSTEM_MOTOR, failure_type, MOTOR_INSTANCE)
    tester.sleep_for(5)
    tester.ensure_motors_stopped(stopped_mask, NUM_MOTORS)

    # Clearing the failure restores all motors.
    tester.inject_failure(FailureUnit.SYSTEM_MOTOR, FailureType.OK, MOTOR_INSTANCE)
    tester.sleep_for(5)
    tester.ensure_motors_stopped(0, NUM_MOTORS)

    tester.stop_checking_altitude()

    tester.execute_rtl()
    tester.wait_until_disarmed(180)


@pytest.mark.motor_failure_injection
def test_detected_motor_failure_removes_motor_and_opposite(tester: AutopilotTester) -> None:
    """Detected motor failure (`failure motor off`): the motor is reported as failed, so the
    control allocator removes it and, with CA_FAILURE_MODE = 1, also stops the geometrically
    opposite motor. Runs on the SIH hexarotor, where motor i's opposite is motor i+3."""
    take_off_and_hover_for_motor_failure(tester)
    # Motor 0 and its opposite, motor 3 (0-based).
    fail_and_restore_motor(tester, FailureType.OFF, 0b1001)


@pytest.mark.motor_failure_injection
def test_undetected_motor_failure_stops_only_failed_motor(tester: AutopilotTester) -> None:
    """Undetected motor failure (`failure motor wrong`): only the injected motor stops, the
    allocator is not informed and the opposite motor keeps running."""
    take_off_and_hover_for_motor_failure(tester)
    fail_and_restore_motor(tester, FailureType.WRONG, 1 << (MOTOR_INSTANCE - 1))


@pytest.mark.motor_failure_injection
def test_return_lands_with_failed_motor_and_heading_off(tester: AutopilotTester) -> None:
    """Return with the motor still failed. With CA_FAILURE_MODE = 1 the hexarotor flies on four
    motors and has no yaw authority left. The vehicle faces 90 deg away from the home
    heading when Return is commanded: navigator must not wait for that heading forever but
    continue after MIS_YAW_TMT and land."""
    take_off_and_hover_for_motor_failure(tester)

    # turn away from the heading Return will command at the destination
    tester.offboard_goto(PositionNedYaw(0.0, 0.0, -FLIGHT_ALTITUDE_M, 90.0),
                         acceptance_radius_m=1.0)
    tester.sleep_for(5)
    tester.hold()

    tester.inject_failure(FailureUnit.SYSTEM_MOTOR, FailureType.OFF, MOTOR_INSTANCE)
    tester.sleep_for(5)
    tester.ensure_motors_stopped(0b1001, NUM_MOTORS)

    tester.execute_rtl()
    tester.wait_until_disarmed(180)


def take_off_and_hover(tester: AutopilotTester) -> None:
    tester.set_takeoff_altitude(FLIGHT_ALTITUDE_M)
    # Necessary for the takeoff altitude to be applied properly.
    tester.sleep_for(1)

    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(FLIGHT_ALTITUDE_M, 30)
    tester.wait_until_speed_lower_than(HOVER_SPEED_TOLERANCE_M_S, 30)
    # Both waits pass while the vehicle is still slowing down at the top of the climb. Let it
    # settle, so the altitude reference for the failure is the hover altitude, not mid-climb.
    tester.sleep_for(5)


def imu_std(tester: AutopilotTester, unit: FailureUnit) -> List[float]:
    """Per-axis standard deviation of the IMU data over IMU_WINDOW_S."""
    samples = tester.record_scaled_imu(IMU_WINDOW_S)
    if unit == FailureUnit.SENSOR_GYRO:
        axes = [(s.angular_velocity_frd.forward_rad_s, s.angular_velocity_frd.right_rad_s,
                 s.angular_velocity_frd.down_rad_s) for s in samples]
    else:
        axes = [(s.acceleration_frd.forward_m_s2, s.acceleration_frd.right_m_s2,
                 s.acceleration_frd.down_m_s2) for s in samples]
    return [statistics.pstdev(axis) for axis in zip(*axes)]


@pytest.mark.imu_failure_injection
@pytest.mark.parametrize("unit", [pytest.param(FailureUnit.SENSOR_GYRO, id="gyro"),
                                  pytest.param(FailureUnit.SENSOR_ACCEL, id="accel")])
def test_hold_altitude_through_imu_failure(tester: AutopilotTester, unit: FailureUnit) -> None:
    """IMU bias, noise and drift (`failure gyro|accel wrong|garbage|drift`) in turn, with the
    default SYS_FAIL_{G,A}_* magnitudes: the vehicle must hold altitude throughout.

    The noise is also checked in the IMU data, which shows that the injection reaches it. Bias
    and drift are not: the controllers react to them and partly hide them in flight."""
    tester.wait_until_ready()
    take_off_and_hover(tester)
    tester.start_checking_altitude(ALTITUDE_TOLERANCE_M)

    healthy = imu_std(tester, unit)
    tester.inject_failure(unit, FailureType.GARBAGE, IMU_INSTANCE)
    noisy = imu_std(tester, unit)
    tester.inject_failure(unit, FailureType.OK, IMU_INSTANCE)
    cleared = imu_std(tester, unit)
    for axis in range(3):
        assert noisy[axis] > 2 * healthy[axis] > cleared[axis], \
            "Axis {}: std healthy {:.4f}, noisy {:.4f}, cleared {:.4f}".format(
                axis, healthy[axis], noisy[axis], cleared[axis])

    tester.inject_failure(unit, FailureType.WRONG, IMU_INSTANCE)
    tester.sleep_for(IMU_FAILURE_DURATION_S)
    tester.inject_failure(unit, FailureType.OK, IMU_INSTANCE)

    tester.inject_failure(unit, FailureType.DRIFT, IMU_INSTANCE)
    tester.sleep_for(IMU_FAILURE_DURATION_S)
    tester.inject_failure(unit, FailureType.OK, IMU_INSTANCE)
    tester.sleep_for(5)

    tester.stop_checking_altitude()
    tester.execute_rtl()
    tester.wait_until_disarmed(180)
