# Streaming MAVLink Messages

This tutorial demonstrates how to stream a uORB message as a MAVLink message, and applies to both standard and custom messages.

## 개요

[MAVLink messages](../middleware/mavlink.md) are streamed using a streaming class, derived from `MavlinkStream`, that has been added to the PX4 stream list.
The class has framework methods that you implement so PX4 can get information it needs from the generated MAVLink message definition.
It also has a `send()` method that is called each time the message needs to be sent — you override this to copy information from a uORB subscription to the MAVLink message object that is to be sent.

Once you have created a streaming class the corresponding message can be streamed on request.
You can also configure PX4 to stream the message by default, depending on the MAVLink configuration.

## 전제 조건

Generally you will already have a [uORB](../middleware/uorb.md) message that contains information you'd like to stream and a definition of a MAVLink message that you'd like to stream it with.

For this example we're going to assume that you want to stream the (existing) [BatteryStatus](../msg_docs/BatteryStatus.md) uORB message to a new MAVLink battery status message, which we will name `BATTERY_STATUS_DEMO`.

Copy this `BATTERY_STATUS_DEMO` message into the message section of `development.xml` in your PX4 source code, which will be located at: `\src\modules\mavlink\mavlink\message_definitions\v1.0\development.xml`.

```xml
    <message id="11514" name="BATTERY_STATUS_DEMO">
      <description>Simple demo battery.</description>
      <field type="uint8_t" name="id" instance="true">Battery ID</field>
      <field type="int16_t" name="temperature" units="cdegC" invalid="INT16_MAX">Temperature of the whole battery pack (not internal electronics). INT16_MAX field not provided.</field>
      <field type="uint8_t" name="percent_remaining" units="%" invalid="UINT8_MAX">Remaining battery energy. Values: [0-100], UINT8_MAX: field not provided.</field>
    </message>
```

:::info
Note that this is a cut-down version of the not-yet-implemented [BATTERY_STATUS_V2](https://mavlink.io/en/messages/development.html#BATTERY_STATUS_V2) message with randomly chosen unused id of `11514`.
Here we've put the message in `development.xml`, which is fine for testing and if the message is intended to eventually be part of the standard message set, but you might also put a [custom message](../mavlink/custom_messages.md) in its own dialect file.
:::

Build PX4 for SITL and confirm that the associated message is generated in `/build/px4_sitl_default/mavlink/development/mavlink_msg_battery_status_demo.h`.

Because `BatteryStatus` already exists you will not need to do anything to create or build it.

## Define the Streaming Class

First create a file named `BATTERY_STATUS_DEMO.hpp` for your streaming class (named after the message to stream) inside the [/src/modules/mavlink/streams](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mavlink/streams) directory.

Add the headers for the uORB message(s) to the top of the file (the required MAVLink headers should already be available):

```cpp
#include <uORB/topics/battery_status.h>
```

:::info
The uORB topic's snake-case header file is generated from the CamelCase uORB filename at build time.
:::

Then copy the streaming class definition below into the file:

```cpp
class MavlinkStreamBatteryStatusDemo : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamBatteryStatusDemo(mavlink);
    }
    const char *get_name() const
    {
        return MavlinkStreamBatteryStatusDemo::get_name_static();
    }
    static const char *get_name_static()
    {
        return "BATTERY_STATUS_DEMO";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_BATTERY_STATUS_DEMO;
    }
    uint16_t get_id()
    {
        return get_id_static();
    }
    unsigned get_size()
    {
        return MAVLINK_MSG_ID_BATTERY_STATUS_DEMO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    //Subscription to array of uORB battery status instances
    uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};
    // SubscriptionMultiArray subscription is needed because battery has multiple instances.
    // uORB::Subscription is used to subscribe to a single-instance topic

    /* do not allow top copying this class */
    MavlinkStreamBatteryStatusDemo(MavlinkStreamBatteryStatusDemo &);
    MavlinkStreamBatteryStatusDemo& operator = (const MavlinkStreamBatteryStatusDemo &);

protected:
    explicit MavlinkStreamBatteryStatusDemo(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

	bool send() override
	{
		bool updated = false;

		// Loop through _battery_status_subs (subscription to array of BatteryStatus instances)
		for (auto &battery_sub : _battery_status_subs) {
            // battery_status_s is a struct that can hold the battery object topic
			battery_status_s battery_status;

			// Update battery_status and publish only if the status has changed
			if (battery_sub.update(&battery_status)) {
                // mavlink_battery_status_demo_t is the MAVLink message object
				mavlink_battery_status_demo_t bat_msg{};

				bat_msg.id = battery_status.id - 1;
				bat_msg.percent_remaining = (battery_status.connected) ? roundf(battery_status.remaining * 100.f) : -1;

				// check if temperature valid
				if (battery_status.connected && PX4_ISFINITE(battery_status.temperature)) {
					bat_msg.temperature = battery_status.temperature * 100.f;
				} else {
					bat_msg.temperature = INT16_MAX;
				}

                //Send the message
				mavlink_msg_battery_status_demo_send_struct(_mavlink->get_channel(), &bat_msg);
				updated = true;
			}
		}

		return updated;
	}

};
```

Most streaming classes are very similar (see examples in [/src/modules/mavlink/streams](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mavlink/streams)):

- The streaming class derives from [`MavlinkStream`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_stream.h) and is named using the pattern `MavlinkStream<CamelCaseMessageName>`.

- The `public` definitions are "near-boilerplate", allowing PX4 to get an instance of the class (`new_instance()`), and then to use it to fetch the name, id, and size of the message from the MAVLink headers (`get_name()`, `get_name_static()`, `get_id_static()`, `get_id()`, `get_size()`).
  For your own streaming classes these can just be copied and modified to match the values for your MAVLink message.

- The `private` definitions subscribe to the uORB topics that need to be published.
  In this case the uORB topic has multiple instances: one for each battery.
  We use `uORB::SubscriptionMultiArray` to get an array of battery status subscriptions.

  Here we also define constructors to prevent the definition being copied.

- The `protected` section is where the important work takes place!

  Here we override the `send()` method, copying values from the subscribed uORB topic(s) into appropriate fields in the MAVLink message, and then send the message.

  In this particular example we have an array of uORB instances `_battery_status_subs` (because we have multiple batteries).
  We iterate the array and use `update()` on each subscription to check if the associated battery instance has changed (and update a structure with the current data).
  This allows us to send the MAVLink message _only_ if the associated battery uORB topic has changed:

    // Struct to hold current topic data.
    battery_status_s battery_status;
    
    // update() populates battery_status and returns true if the status has changed
    if (battery_sub.update(&battery_status)) {
       // Use battery_status to populate message and send
    }

  If wanted to send a MAVLink message whether or not the data changed, we could instead use `copy()` as shown:

    battery_status_s battery_status;
    battery_sub.copy(&battery_status);

  ::: info
  For a single-instance topic like [VehicleStatus](../msg_docs/VehicleStatus.md) we would subscribe like this:

    // Create subscription _vehicle_status_sub
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

  And we could use the resulting subscription in the same way with update or copy.

    vehicle_status_s vehicle_status{}; // vehicle_status_s is the definition of the uORB topic
    if (_vehicle_status_sub.update(&vehicle_status)) {
      // Use the vehicle_status as it has been updated.
    }


:::

Next we include our new class in [mavlink_messages.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_messages.cpp#L2193).
Add the line below to the part of the file where all the other streams are included:

```cpp
#include "streams/BATTERY_STATUS_DEMO.hpp"
```

Finally append the stream class to the `streams_list` at the bottom of
[mavlink_messages.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_messages.cpp)

```C
StreamListItem *streams_list[] = {
...
#if defined(BATTERY_STATUS_DEMO_HPP)
    create_stream_list_item<MavlinkStreamBatteryStatusDemo>(),
#endif // BATTERY_STATUS_DEMO_HPP
...
}
```

The class is now available for streaming, but won't be streamed by default.
We cover that in the next sections.

## Streaming by Default

The easiest way to stream your messages by default (as part of a build) is to add them to [mavlink_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_main.cpp) in the appropriate message group.

If you search in the file you'll find groups of messages defined in a switch statement:

- `MAVLINK_MODE_NORMAL`: Streamed to a GCS.
- `MAVLINK_MODE_ONBOARD`: Streamed to a companion computer on a fast link, such as Ethernet
- `MAVLINK_MODE_ONBOARD_LOW_BANDWIDTH`: Streamed to a companion computer for re-routing to a reduced-traffic link, such as a GCS.
- `MAVLINK_MODE_GIMBAL`: Streamed to a gimbal
- `MAVLINK_MODE_EXTVISION`: Streamed to an external vision system
- `MAVLINK_MODE_EXTVISIONMIN`: Streamed to an external vision system on a slower link
- `MAVLINK_MODE_OSD`: Streamed to an OSD, such as an FPV headset.
- `MAVLINK_MODE_CUSTOM`: Stream nothing by default. Used when configuring streaming using MAVLink.
- `MAVLINK_MODE_MAGIC`: Same as `MAVLINK_MODE_CUSTOM`
- `MAVLINK_MODE_CONFIG`: Streaming over USB with higher rates than `MAVLINK_MODE_NORMAL`.
- `MAVLINK_MODE_MINIMAL`: Stream a minimal set of messages. Normally used for poor telemetry links.
- `MAVLINK_MODE_IRIDIUM`: Streamed to an iridium satellite phone

Normally you'll be testing on a GCS, so you could just add the message to the `MAVLINK_MODE_NORMAL` case using the `configure_stream_local()` method.
For example, to stream CA_TRAJECTORY at 5 Hz:

```cpp
	case MAVLINK_MODE_CONFIG: // USB
		// Note: streams requiring low latency come first
		...
		configure_stream_local("BATTERY_STATUS_DEMO", 5.0f);
        ...
```

It is also possible to add a stream by calling the [mavlink](../modules/modules_communication.md#mavlink) module with the `stream` argument in a [startup script](../concept/system_startup.md).
For example, you might add the following line to [/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink) in order to stream `BATTERY_STATUS_DEMO` at 50Hz on UDP port `14556` (`-r` configures the streaming rate and `-u` identifies the MAVLink channel on UDP port 14556).

```sh
mavlink stream -r 50 -s BATTERY_STATUS_DEMO -u 14556
```

## Streaming on Request

Some messages are only needed once, when particular hardware is connected, or under other circumstances.
In order to avoid clogging communications links with messages that aren't needed you may not stream all messages by default, even at low rate.

If you needed, a GCS or other MAVLink API can request that particular messages are streamed at a particular rate using [MAV_CMD_SET_MESSAGE_INTERVAL](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL).
A particular message can be requested just once using [MAV_CMD_REQUEST_MESSAGE](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE).

