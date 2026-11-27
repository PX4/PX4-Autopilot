import math

import pytest

from autopilot_tester import (
    ACTUATOR_SET_INT_SCALE, INT32_MAX, MAV_CMD_DO_SET_ACTUATOR, MAV_FRAME_MISSION, AutopilotTester)

# PWM outputs unused by the SIH quad.
ACTUATOR5_OUTPUT = 5
ACTUATOR6_OUTPUT = 6


@pytest.mark.multicopter
def test_command_int_set_actuator_single_unset_param(tester: AutopilotTester) -> None:
    """A COMMAND_INT DO_SET_ACTUATOR leaves an actuator unchanged when only its own x/y is unset.

    Only x and y both unset used to be ignored, so setting just actuator 5 (y = INT32_MAX)
    drove actuator 6 to saturation.
    """
    tester.wait_until_ready()
    tester.route_actuator_set_to_output(5, ACTUATOR5_OUTPUT)
    tester.route_actuator_set_to_output(6, ACTUATOR6_OUTPUT)
    tester.enable_actuator_output_status()
    tester.arm()
    # Never set, so it outputs its disarmed value.
    actuator6_us = tester.wait_for_actuator_output_value(ACTUATOR6_OUTPUT - 1, 5)

    actuator5 = 0.5
    tester.send_command_int(MAV_CMD_DO_SET_ACTUATOR, param1=math.nan, param2=math.nan,
                            param3=math.nan, param4=math.nan,
                            x=round(actuator5 * ACTUATOR_SET_INT_SCALE), y=INT32_MAX, z=0.0,
                            frame=MAV_FRAME_MISSION)

    tester.wait_for_actuator_set_output(ACTUATOR5_OUTPUT, actuator5, 5)
    tester.wait_for_actuator_output(ACTUATOR6_OUTPUT - 1, actuator6_us, 1)
