import json
from typing import Any, Dict, Tuple

import pytest
from mavsdk import Configuration, ConnectionResult, Mavsdk  # type: ignore
from mavsdk.plugins.mavlink_direct_server import (  # type: ignore
    MavlinkDirectServer, MavlinkMessage)

from autopilot_tester import AutopilotTester, MAV_COMP_ID_AUTOPILOT1

MAV_COMP_ID_BATTERY = 180
MAVLINK_MSG_ID_BATTERY_STATUS = 147
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_BATTERY_FUNCTION_ALL = 1
MAV_BATTERY_FUNCTION_PAYLOAD = 4
TEMPERATURE_UNKNOWN = 32767


@pytest.mark.multicopter
def test_mavlink_batteries_keep_ids_and_instances(tester: AutopilotTester) -> None:
    system_id = tester._target_system_id()
    observer = tester._mavlink_direct
    temperatures: Dict[Tuple[int, int], int] = {}

    def on_message(message: Any, _user_data: Any) -> None:
        if message.system_id == system_id:
            fields = json.loads(message.fields_json)
            temperatures[message.component_id, fields["id"]] = fields["temperature"]

    handle = observer.subscribe_message("BATTERY_STATUS", on_message)
    try:
        assert tester.poll_condition_with_timeout(
            lambda: (MAV_COMP_ID_AUTOPILOT1, 0) in temperatures, 20), "No onboard battery"

        # Stop the stream while sending, then restart it to read each battery.
        # Otherwise one overwritten instance could still report both IDs in turn.
        tester.send_command_long(
            MAV_CMD_SET_MESSAGE_INTERVAL,
            param1=MAVLINK_MSG_ID_BATTERY_STATUS, param2=-1)

        configuration = Configuration.create_manual(system_id, MAV_COMP_ID_BATTERY, False)
        with Mavsdk(configuration) as battery_sdk:
            # Payload link from init.d-posix/px4-rc.mavlink, SITL instance 0.
            assert battery_sdk.add_any_connection(
                "udpout://127.0.0.1:14280") == ConnectionResult.SUCCESS
            sender = MavlinkDirectServer(
                battery_sdk.server_component_by_id(MAV_COMP_ID_BATTERY))

            # Send the payload first so it cannot be ignored just because all slots are full.
            for battery_id, function, temperature in (
                (122, MAV_BATTERY_FUNCTION_PAYLOAD, 2600),
                (120, MAV_BATTERY_FUNCTION_ALL, 2600),
                (121, MAV_BATTERY_FUNCTION_ALL, TEMPERATURE_UNKNOWN),
            ):
                sender.send_message(MavlinkMessage(
                    message_name="BATTERY_STATUS", fields_json=json.dumps({
                        "id": battery_id,
                        "battery_function": function,
                        "temperature": temperature,
                        "voltages": [4000] * 4 + [65535] * 6,
                        "battery_remaining": 0 if battery_id == 122 else 80,
                    })))

            assert tester.poll_condition_with_timeout(
                lambda: all((MAV_COMP_ID_BATTERY, i) in temperatures
                            for i in (120, 121, 122)), 20), \
                "Battery messages were not forwarded"

            temperatures.pop((MAV_COMP_ID_AUTOPILOT1, 0))
            tester.send_command_long(
                MAV_CMD_SET_MESSAGE_INTERVAL,
                param1=MAVLINK_MSG_ID_BATTERY_STATUS, param2=0)
            assert tester.poll_condition_with_timeout(
                lambda: all((MAV_COMP_ID_AUTOPILOT1, i) in temperatures
                            for i in (0, 120, 121)), 20), \
                "Autopilot did not report all three battery IDs"

            assert temperatures[MAV_COMP_ID_AUTOPILOT1, 120] == 2600
            assert temperatures[MAV_COMP_ID_AUTOPILOT1, 121] == TEMPERATURE_UNKNOWN
            assert (MAV_COMP_ID_AUTOPILOT1, 122) not in temperatures
    finally:
        observer.unsubscribe_message(handle)
