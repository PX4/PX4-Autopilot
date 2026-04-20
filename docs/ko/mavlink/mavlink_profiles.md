# MAVLink Profiles

A MAVLink _profile_ (also called a _mode_) defines a set of messages that can be streamed by default on a MAVLink channel and their rates.

This section lists the profiles, and explains how they can be used and extended.

## Available Profiles

The available profiles (in source-code declaration order) are:

- _Normal_ (`MAVLINK_MODE_NORMAL`): Set of messages for a GCS.
- _Onboard_ (`MAVLINK_MODE_ONBOARD`): Set of messages for a companion computer on a fast link (such as Ethernet).
- _Gimbal_ (`MAVLINK_MODE_GIMBAL`): Messages for a gimbal. Note this also enables message forwarding.
- _External Vision_ (`MAVLINK_MODE_EXTVISION`): Messages for offboard vision systems.
- _External Vision Minimal_ (`MAVLINK_MODE_EXTVISIONMIN`): Messages for offboard vision systems on slower links.
- _OSD_ (`MAVLINK_MODE_OSD`): Set of messages for an OSD system.
- _Magic_ (`MAVLINK_MODE_MAGIC`): No messages streamed by default. Used when configuring streaming dynamically via MAVLink.
- _Custom_ (`MAVLINK_MODE_CUSTOM`): Same as `MAVLINK_MODE_MAGIC`.
- _Config_ (`MAVLINK_MODE_CONFIG`): Set of messages for configuration interface, sent at higher rates. This is used, for example, to send the `MAVLINK_MODE_NORMAL` message set via USB to a GCS.
- _Iridium_ (`MAVLINK_MODE_IRIDIUM`): Streams `HIGH_LATENCY2` message to an iridium satellite phone.
- _Minimal_ (`MAVLINK_MODE_MINIMAL`): Minimal set of messages for use with a GCS on a poor telemetry link.
- _Onboard Low Bandwidth_ (`MAVLINK_MODE_ONBOARD_LOW_BANDWIDTH`): Set of messages to be streamed to a companion computer for re-routing to a reduced-traffic link, such as a GCS.
- _Low Bandwidth_ (`MAVLINK_MODE_LOW_BANDWIDTH`): Reduced message set for low bandwidth links.
- _uAvionix_ (`MAVLINK_MODE_UAVIONIX`): Messages for a uAvionix ADS-B beacon.
- _Distance Sensor_ (`MAVLINK_MODE_DISTANCE_SENSOR`): Streams distance sensor data at unlimited rate.

:::tip
The profile defines the _default_ messages and rates.
A connected MAVLink system can still request the streams/rates it wants using [MAV_CMD_SET_MESSAGE_INTERVAL](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL).
:::

To find the exact messages in each profile, search for ` configure_streams_to_default` (or the above profile names) in [mavlink_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_main.cpp).

## Assigning Profiles to Ports

[MAVLink Peripherals](../peripherals/mavlink_peripherals.md) explains how to set up a port for communicating over MAVLink.
This uses the concept of an abstract [MAVLink instance](../peripherals/mavlink_peripherals.md#mavlink-instances) which is then assigned to a serial port.

The profile associated with a particular MAVLink instance is set using the associated `MAV_X_MODE` parameter:

- [MAV_0_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE)
- [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE)
- [MAV_2_MODE](../advanced_config/parameter_reference.md#MAV_2_MODE)

There are also dedicated profile parameters for ports that are not configured via MAVLink instances:

- [USB_MAV_MODE](../advanced_config/parameter_reference.md#USB_MAV_MODE): Profile for the USB port (used when MAVLink is set or detected on USB).
- [MAV_S_MODE](../advanced_config/parameter_reference.md#MAV_S_MODE): Profile for the internal SOM (System on Module) to FMU communication channel, used on boards where the FMU and companion computer are co-located on the same module.

Note that not all profiles can necessarily be set on these ports.

## Adding Messages to a Profile

You can add messages to a profile in appropriate `case` switch in the [Mavlink::configure_streams_to_default(const char \*configure_single_stream)](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_main.cpp#L1430) method (see [mavlink_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_main.cpp)).

If you're testing with a GCS over USB you might add the message to the `MAVLINK_MODE_CONFIG` case using the `configure_stream_local()` method.
For example, to stream `BATTERY_STATUS_DEMO` at 5 Hz:

```cpp
	case MAVLINK_MODE_CONFIG: // USB
		// Note: streams requiring low latency come first
		...
		configure_stream_local("BATTERY_STATUS_DEMO", 5.0f);
        ...
```

See [Streaming MAVLink Messages](streaming_messages.md) for a more detailed example.
