"""Control allocation with motor failures, on the gazebo-classic typhoon_h480 hexarotor."""

import math

import pytest
from mavsdk.plugins.failure import FailureType, FailureUnit  # type: ignore

from autopilot_tester import AutopilotTester, MissionOptions

FLIGHT_ALTITUDE_M = 10.0
ALTITUDE_TOLERANCE_M = 4.0
HOVER_SPEED_TOLERANCE_M_S = 1.0
NUM_MOTORS = 6  # TODO: get from the model


def configure_failure_handling(tester: AutopilotTester) -> None:
    tester.set_param_int("SYS_FAILURE_EN", 1)  # failure injection
    tester.set_param_int("FD_ACT_EN", 1)  # motor failure detection
    tester.set_param_int("MC_AIRMODE", 1)  # airmode for control allocation with a failed motor
    tester.set_param_int("CA_FAILURE_MODE", 1)  # control allocation handles the failed motor


def prepare_mission_and_take_off(tester: AutopilotTester,
                                 enable_actuator_output_status: bool = False) -> None:
    tester.wait_until_ready()
    configure_failure_handling(tester)
    tester.prepare_square_mission(
        MissionOptions(rtl_at_end=False, relative_altitude_m=FLIGHT_ALTITUDE_M))
    tester.set_takeoff_altitude(FLIGHT_ALTITUDE_M)
    tester.set_rtl_altitude(FLIGHT_ALTITUDE_M)
    tester.check_tracks_mission(5.0)
    tester.store_home()
    if enable_actuator_output_status:
        tester.enable_actuator_output_status()
    # Necessary for the takeoff altitude to be applied properly.
    tester.sleep_for(1)

    take_off_and_hover(tester, FLIGHT_ALTITUDE_M, 30)


def take_off_and_hover(tester: AutopilotTester, altitude_m: float, timeout_s: float) -> None:
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(altitude_m, timeout_s)
    tester.wait_until_speed_lower_than(HOVER_SPEED_TOLERANCE_M_S, timeout_s)


def fail_motor(tester: AutopilotTester, instance: int, failure_type: FailureType) -> None:
    tester.inject_failure(FailureUnit.SYSTEM_MOTOR, failure_type, instance)
    tester.sleep_for(1)


@pytest.mark.controlallocation
def test_remove_one_motor(tester: AutopilotTester) -> None:
    motor_instance = 1
    prepare_mission_and_take_off(tester, enable_actuator_output_status=True)

    tester.start_checking_altitude(ALTITUDE_TOLERANCE_M)
    fail_motor(tester, motor_instance, FailureType.OFF)
    tester.ensure_motors_stopped(1 << (motor_instance - 1), NUM_MOTORS)

    tester.execute_mission()
    tester.stop_checking_altitude()
    tester.ensure_motors_stopped(1 << (motor_instance - 1), NUM_MOTORS)

    tester.execute_rtl()
    tester.wait_until_disarmed(180)
    tester.check_home_within(5.0)


@pytest.mark.controlallocation
def test_remove_two_motors(tester: AutopilotTester) -> None:
    prepare_mission_and_take_off(tester)

    # Two motors opposite of one another on the hexa airframe.
    tester.start_checking_altitude(ALTITUDE_TOLERANCE_M)
    fail_motor(tester, 1, FailureType.OFF)
    fail_motor(tester, 2, FailureType.OFF)

    tester.execute_mission()
    tester.stop_checking_altitude()

    # Return with two motors out doesn't work, because navigator waits forever for the yaw
    # setpoint to be reached during Return, so it never lands.
    tester.land()
    tester.wait_until_disarmed(180)


@pytest.mark.controlallocation
def test_remove_and_restore_every_motor_once(tester: AutopilotTester) -> None:
    prepare_mission_and_take_off(tester)

    tester.start_checking_altitude(ALTITUDE_TOLERANCE_M)
    for motor_instance in range(1, NUM_MOTORS + 1):
        fail_motor(tester, motor_instance, FailureType.OFF)
        fail_motor(tester, motor_instance, FailureType.OK)

    tester.execute_mission()
    tester.stop_checking_altitude()

    tester.execute_rtl()
    tester.wait_until_disarmed(180)
    tester.check_home_within(5.0)


@pytest.mark.controlallocation
def test_return_home_on_motor_failure(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    configure_failure_handling(tester)
    tester.set_param_int("COM_ACT_FAIL_ACT", 3)  # Return on motor failure
    tester.set_takeoff_altitude(FLIGHT_ALTITUDE_M)
    tester.store_home()
    tester.sleep_for(1)

    take_off_and_hover(tester, FLIGHT_ALTITUDE_M, 30)

    # TODO: fly forward a bit before the motor failure to distinguish "Return" from "Land only".
    fail_motor(tester, 1, FailureType.OFF)

    # Return triggers automatically.
    tester.wait_until_disarmed(180)
    tester.check_home_within(5.0)


@pytest.mark.controlallocation
def test_terminate_on_motor_failure(tester: AutopilotTester) -> None:
    flight_altitude_m = 100.0

    tester.wait_until_ready()
    configure_failure_handling(tester)
    tester.set_param_int("COM_ACT_FAIL_ACT", 4)  # Terminate on motor failure
    tester.set_takeoff_altitude(flight_altitude_m)
    tester.sleep_for(1)

    tester.arm()
    tester.takeoff()
    tester.wait_until_altitude(flight_altitude_m, 60)
    tester.wait_until_speed_lower_than(HOVER_SPEED_TOLERANCE_M_S, 60)

    fail_motor(tester, 1, FailureType.OFF)

    # The timeout is low enough that the drone has to free fall (terminate) to disarm in time:
    # h = g/2 * t^2, solved for t, plus some margin.
    seconds_to_touchdown = int(2 + math.sqrt(flight_altitude_m * 2 / 10.0))
    tester.wait_until_disarmed(seconds_to_touchdown)
