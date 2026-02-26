# Custom MAVLink Messages

A custom [MAVLink message](../middleware/mavlink.md) is one that isn't in the standard MAVLink definitions that are included into PX4 by default.

:::info
If you use a custom definition you will fork and maintain PX4, your ground station, and any other SDKs that communicate with it.
Generally you should use (or add to) the standard definitions if at all possible to reduce the maintenance burden.
:::

## Adding Custom XML

Custom definitions can be added in a new dialect file in the same directory as [when using the standard XML definitions](../mavlink/adding_messages.md).
For example, create `PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0/custom_messages.xml`, and set `CONFIG_MAVLINK_DIALECT` to build the new file for SITL.
This dialect file should include `development.xml` so that all the standard definitions are also included.

For initial prototyping, or if you intend your message to be "standard", you can also add your messages to `common.xml` (or `development.xml`).
This simplifies building, because you don't need to modify the dialect that is built.

The MAVLink developer guide explains how to define new messages in [How to Define MAVLink Messages & Enums](https://mavlink.io/en/guide/define_xml_element.html).

You can check that your new messages are built by inspecting the headers generated in the build directory (`/build/<build target>/mavlink/`).
If your messages are not built they may be incorrectly formatted, or use clashing ids.
Inspect the build log for information.

Once the message is being built you can stream, receive, or otherwise use it, as described in the following sections.

:::info
The [MAVLink Developer guide](https://mavlink.io/en/getting_started/) has more information about using the MAVLink toolchain.
:::

## 另一种自定义MAVlink消息的办法

Sometimes there is the need for a custom MAVLink message with content that is not fully defined.

For example when using MAVLink to interface PX4 with an embedded device, the messages that are exchanged between the autopilot and the device may go through several iterations before they are stabilized.
In this case, it can be time-consuming and error-prone to regenerate the MAVLink headers, and make sure both devices use the same version of the protocol.

An alternative - and temporary - solution is to re-purpose debug messages.
Instead of creating a custom MAVLink message `CA_TRAJECTORY`, you can send a message `DEBUG_VECT` with the string key `CA_TRAJ` and data in the `x`, `y` and `z` fields.
See [this tutorial](../debug/debug_values.md) for an example usage of debug messages.

:::info
This solution is not efficient as it sends character string over the network and involves comparison of strings.
It should be used for development only!
:::

## Testing & Updating Ground Stations

Testing the code and updating ground stations is done in the same way as when [Adding New Standard MAVLink Definitions ](../mavlink/adding_messages.md).
