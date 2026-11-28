# Receiving MAVLink Messages

This topic explains how to receive a [MAVLink message](../middleware/mavlink.md) and publish it to uORB.

## Overview

The topic shows how we would handle a received `BATTERY_STATUS_DEMO` message (as defined in [Streaming MAVLink Messages](../mavlink/streaming_messages.md)) and then update the (existing) [BatteryStatus uORB message](../msg_docs/BatteryStatus.md) with the contained information.

This is the kind of implementation that you would provide to support a MAVLink battery integration with PX4.

## Steps

Add the headers for the uORB topic to publish to in [mavlink_receiver.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.h#L77):

```cpp
#include <uORB/topics/battery_status.h>
```

Add a function signature for a function that handles the incoming MAVLink message in the `MavlinkReceiver` class in
[mavlink_receiver.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.h#L126)

```cpp
void handle_message_battery_status_demo(mavlink_message_t *msg);
```

Use the existing `publish_battery_status()` helper to map each MAVLink battery to a separate uORB instance shared across MAVLink links.
Convert the zero-based MAVLink battery ID to the one-based uORB ID before calling it.

Implement the `handle_message_battery_status_demo` function in [mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.cpp).

```cpp
void
MavlinkReceiver::handle_message_battery_status_demo(mavlink_message_t *msg)
{
	if ((msg->sysid != mavlink_system.sysid) || (msg->compid == mavlink_system.compid)) {
		// ignore battery status coming from other systems or from the autopilot itself
		return;
	}

	// external battery measurements
	mavlink_battery_status_t battery_mavlink;
	mavlink_msg_battery_status_decode(msg, &battery_mavlink);

	if (battery_mavlink.id == UINT8_MAX || battery_mavlink.battery_function == MAV_BATTERY_FUNCTION_PAYLOAD) {
		return;
	}

	battery_status_s battery_status{};
	battery_status.timestamp = hrt_absolute_time();
	battery_status.id = battery_mavlink.id + 1;

	battery_status.remaining = (float)battery_mavlink.battery_remaining / 100.0f;
	battery_status.temperature = (battery_mavlink.temperature == INT16_MAX) ?
				     NAN : (float)battery_mavlink.temperature / 100.0f;
	battery_status.connected = true;
	battery_status.source = battery_status_s::SOURCE_EXTERNAL;

	publish_battery_status(msg->compid, battery_status);
}
```

::: info
Above we only write to the battery fields that are defined in the topic.
In practice you'd update all fields with either valid or invalid values: this has been cut back for brevity.
A connected MAVLink battery that stops reporting becomes unhealthy after five seconds; its uORB instance is retained to detect that loss.
Reports from the same component and battery ID clear the timeout without a reboot.
:::

::: tip
If the message has a `target_system` field, don't compare the decoded field with `mavlink_system.sysid` directly.
MAVLink system IDs can be 32 bit: a target above 255 is sent in a header extension and the 8-bit payload field only holds 255.
Use `mavlink_target_system(msg, decoded.target_system)` to get the full target instead.
When sending a message to a system, use `mavlink_msg_send_struct_target()` instead of `mavlink_msg_<name>_send_struct()` so a target above 255 is not truncated.
:::

and finally make sure it is called in [MavlinkReceiver::handle_message()](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.cpp#L228)

```cpp
MavlinkReceiver::handle_message(mavlink_message_t *msg)
 {
    switch (msg->msgid) {
        ...
    case MAVLINK_MSG_ID_BATTERY_STATUS_DEMO:
        handle_message_battery_status_demo(msg);
        break;
        ...
    }
 }
```
