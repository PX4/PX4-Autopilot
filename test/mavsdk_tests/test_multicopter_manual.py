import pytest
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import AutopilotTester, MAV_RESULT_TEMPORARILY_REJECTED

MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7


@pytest.mark.multicopter
@pytest.mark.vtol
def test_fly_forward_in_position_control(tester: AutopilotTester) -> None:
    tester.fly_forward_in_posctl()
    tester.wait_until_disarmed(180)
    tester.check_home_not_within(5.0)


@pytest.mark.multicopter
@pytest.mark.vtol
def test_fly_forward_in_altitude_control(tester: AutopilotTester) -> None:
    tester.fly_forward_in_altctl()
    tester.wait_until_disarmed(180)
    tester.check_home_not_within(5.0)


@pytest.mark.multicopter
def test_low_throttle_rtl_to_stabilized_is_rejected(tester: AutopilotTester) -> None:
    """An Auto-to-Stabilized takeover needs a configured safe manual-throttle position."""
    tester.set_param_int("COM_RC_IN_MODE", 1)  # MAVLink manual control
    tester.set_param_float("COM_MAN_THR_MIN", 0.0)
    tester.set_takeoff_altitude(10.0)
    tester.wait_until_ready()
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()

    # MAVSDK manual control is normalized from 0 (minimum) to 1 (maximum) throttle.
    # Stream continuously so the rejection is caused by low throttle, not by manual-control loss.
    with tester.manual_control_stream(0.0, 0.0, 0.0, 0.0) as set_manual_control:
        tester.sleep_for(1.0)

        tester.execute_rtl()
        tester.wait_for_flight_mode(FlightMode.RETURN_TO_LAUNCH, 10)

        tester.send_command_long(MAV_CMD_DO_SET_MODE,
                                 param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 param2=PX4_CUSTOM_MAIN_MODE_STABILIZED,
                                 expected_result=MAV_RESULT_TEMPORARILY_REJECTED)
        tester.sleep_for(1.0)
        tester.wait_for_flight_mode(FlightMode.RETURN_TO_LAUNCH, 1)

        # Centered manual throttle meets the configured threshold and allows the takeover.
        set_manual_control(0.0, 0.0, 0.5, 0.0)
        tester.sleep_for(1.0)
        tester.send_command_long(MAV_CMD_DO_SET_MODE,
                                 param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 param2=PX4_CUSTOM_MAIN_MODE_STABILIZED)
        tester.wait_for_flight_mode(FlightMode.STABILIZED, 5)

        tester.execute_land()
        tester.wait_until_disarmed(90)
