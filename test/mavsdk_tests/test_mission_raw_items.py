import math
from typing import Any

import pytest

from autopilot_tester import (
    ACTUATOR_SET_INT_SCALE, INT32_MAX, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_DO_SET_ACTUATOR,
    MAV_CMD_NAV_LAND, MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT,
    MAV_FRAME_MISSION, AutopilotTester, raw_mission_item)

SPARE_OUTPUT = 5  # PWM output unused by the SIH quad


def _global_item(tester: AutopilotTester, seq: int, command: int, north_m: float, east_m: float,
                 altitude_m: float, current: int = 0) -> Any:
    latitude_deg, longitude_deg = tester.get_coordinate_transformation().global_from_local(
        north_m, east_m)
    return raw_mission_item(seq, command, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            x=round(latitude_deg * 1e7), y=round(longitude_deg * 1e7),
                            z=altitude_m, current=current)


@pytest.mark.multicopter
def test_mission_set_actuator_scaling(tester: AutopilotTester) -> None:
    """DO_SET_ACTUATOR param5/param6 in a mission item are scaled by 1e7, like COMMAND_INT.

    Unscaled, a fractional actuator 5 value arrived as an integer (-1, 0 or 1) and an unset
    (INT32_MAX) one drove the actuator to saturation instead of leaving it unchanged.
    """
    tester.wait_until_ready()
    tester.route_actuator_set_to_output(5, SPARE_OUTPUT)
    tester.enable_actuator_output_status()

    actuator5 = 0.5
    set_actuator = raw_mission_item(2, MAV_CMD_DO_SET_ACTUATOR, MAV_FRAME_MISSION,
                                    x=round(actuator5 * ACTUATOR_SET_INT_SCALE), y=INT32_MAX)
    # Actuators 1 to 4 left unchanged, actuator set index 0.
    set_actuator.param1 = set_actuator.param2 = set_actuator.param3 = set_actuator.param4 = math.nan

    altitude_m = 5.0
    tester.upload_mission_raw([
        _global_item(tester, 0, MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, altitude_m, current=1),
        _global_item(tester, 1, MAV_CMD_NAV_WAYPOINT, 5.0, 0.0, altitude_m),
        set_actuator,
        _global_item(tester, 3, MAV_CMD_NAV_WAYPOINT, 0.0, 0.0, altitude_m),
        _global_item(tester, 4, MAV_CMD_NAV_LAND, 0.0, 0.0, 0.0),
    ])

    downloaded = tester.download_mission_raw()[2]
    assert downloaded.command == MAV_CMD_DO_SET_ACTUATOR
    assert downloaded.x == round(actuator5 * ACTUATOR_SET_INT_SCALE), \
        "actuator 5 downloaded as {}".format(downloaded.x)
    assert downloaded.y == INT32_MAX, "unset actuator 6 downloaded as {}".format(downloaded.y)

    tester.arm()
    tester.start_mission_raw_and_wait_for_sequence(3)
    tester.wait_for_actuator_set_output(SPARE_OUTPUT, actuator5, 5)
    tester.wait_until_disarmed(120)


@pytest.mark.multicopter
def test_mission_global_frame_arm_disarm_accepted(tester: AutopilotTester) -> None:
    """COMPONENT_ARM_DISARM is accepted in a global frame too, not only in MAV_FRAME_MISSION.

    upload_mission_raw() raises if the vehicle rejects the mission.
    """
    tester.wait_until_ready()

    # x/y (latitude/longitude) must be unset: COMPONENT_ARM_DISARM only supports param1/param2.
    arm = raw_mission_item(1, MAV_CMD_COMPONENT_ARM_DISARM, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                           x=INT32_MAX, y=INT32_MAX, z=math.nan, param1=1.0)
    tester.upload_mission_raw([
        _global_item(tester, 0, MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 5.0, current=1),
        arm,
        _global_item(tester, 2, MAV_CMD_NAV_LAND, 0.0, 0.0, 0.0),
    ])
