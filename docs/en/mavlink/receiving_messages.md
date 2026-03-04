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

Normally you would add a uORB publisher for the uORB topic to publish in the `MavlinkReceiver` class in
[mavlink_receiver.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.h#L296).
In this case the [BatteryStatus](../msg_docs/BatteryStatus.md) uORB topic already exists:

```cpp
uORB::Publication<battery_status_s> _battery_pub{ORB_ID(battery_status)};
```

This creates a publication to a single uORB topic instance, which by default will be the _first_ instance.

::: info
This implementation won't work on multi-battery systems, because several batteries might be publishing data to the first instance of the topic, and there is no way to differentiate them.
To support multiple batteries we'd need to use `PublicationMulti` and map the MAVLink message instance IDs to specific uORB topic instances.
:::

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

	battery_status_s battery_status{};
	battery_status.timestamp = hrt_absolute_time();

	battery_status.remaining = (float)battery_mavlink.battery_remaining / 100.0f;
	battery_status.temperature = (float)battery_mavlink.temperature;
	battery_status.connected = true;

	_battery_pub.publish(battery_status);
}
```

::: info
Above we only write to the battery fields that are defined in the topic.
In practice you'd update all fields with either valid or invalid values: this has been cut back for brevity.
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
