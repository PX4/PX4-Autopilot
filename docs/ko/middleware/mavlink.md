# MAVLink 메시징

[MAVLink](https://mavlink.io/en/) is a very lightweight messaging protocol that has been designed for the drone ecosystem.

PX4 uses _MAVLink_ to communicate with ground stations and MAVLink SDKs, such as _QGroundControl_ and [MAVSDK](https://mavsdk.mavlink.io/), and as the integration mechanism for connecting to drone components outside of the flight controller: companion computers, MAVLink enabled cameras, and so on.

This topic provides a brief overview of fundamental MAVLink concepts, such as messages, commands, and microservices.
It also links instructions for how you can add PX4 support for:

- [Adding Standard Messages](../mavlink/adding_messages.md)
- [Streaming MAVLink messages](../mavlink/streaming_messages.md)
- [Handling incoming MAVLink messages (and writing to a uORB topic)](../mavlink/receiving_messages.md)
- [Custom MAVLink Messages](../mavlink/custom_messages.md)

:::info
We do not yet cover _command_ handling and sending, or how to implement your own microservices.
:::

## MAVLink Overview

MAVLink is a lightweight protocol that was designed for efficiently sending messages over unreliable low-bandwidth radio links.

_Messages_ are simplest and most "fundamental" definition in MAVLink, consisting of a name (e.g. [ATTITUDE](https://mavlink.io/en/messages/common.html#ATTITUDE)), id, and fields containing relevant data.
They are deliberately lightweight, with a constrained size, and no semantics for resending and acknowledgement.
Stand-alone messages are commonly used for streaming telemetry or status information, and for sending commands where no acknowledgement is required - such as setpoint commands sent at high rate.

The [Command Protocol](https://mavlink.io/en/services/command.html) is a higher level protocol for sending commands that may need acknowledgement.
Specific commands are defined as values of the [MAV_CMD](https://mavlink.io/en/messages/common.html#mav_commands) enumeration, such as the takeoff command [MAV_CMD_NAV_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF), and include up to 7 numeric "param" values.
The protocol sends a command by packaging the parameter values in a `COMMAND_INT` or `COMMAND_LONG` message, and waits for an acknowledgement with a result in a `COMMAND_ACK`.
The command is resent automatically if no acknowledgment is received.
Note that [MAV_CMD](https://mavlink.io/en/messages/common.html#mav_commands) definitions are also used to define mission actions, and that not all definitions are supported for use in commands/missions on PX4.

[Microservices](https://mavlink.io/en/services/) are other higher level protocols built on top of MAVLink messages.
They are used to communicate information that cannot be sent in a single message, and to deliver features such as reliable communication.
The command protocol described above is one such service.
Others include the [File Transfer Protocol](https://mavlink.io/en/services/ftp.html), [Camera Protocol](https://mavlink.io/en/services/camera.html) and [Mission Protocol](https://mavlink.io/en/services/mission.html).

MAVLink messages, commands and enumerations are defined in [XML definition files](https://mavlink.io/en/guide/define_xml_element.html).
The MAVLink toolchain includes code generators that create programming-language-specific libraries from these definitions for sending and receiving messages.
Note that most generated libraries do not create code to implement microservices.

The MAVLink project standardizes a number of messages, commands, enumerations, and microservices, for exchanging data using the following definition files (note that higher level files _include_ the definitions of the files below them):

- [development.xml](https://mavlink.io/en/messages/development.html) — Definitions that are proposed to be part of the standard.
  The definitions move to `common.xml` if accepted following testing.
- [common.xml](https://mavlink.io/en/messages/common.html) — A "library" of definitions meeting many common UAV use cases.
  These are supported by many flight stacks, ground stations, and MAVLink peripherals.
  Flight stacks that use these definitions are more likely to interoperate.
- [standard.xml](https://mavlink.io/en/messages/standard.html) — Definitions that are actually standard.
  They are present on the vast majority of flight stacks and implemented in the same way.
- [minimal.xml](https://mavlink.io/en/messages/minimal.html) — Definitions required by a minimal MAVLink implementation.

The project also hosts [dialect XML definitions](https://mavlink.io/en/messages/#dialects), which contain MAVLink definitions that are specific to a flight stack or other stakeholder.

The protocol relies on each end of the communication having a shared definition of what messages are being sent.
What this means is that in order to communicate both ends of the communication must use libraries generated from the same XML definition.

<!--
The messages are sent over-the-wire in the "payload" of a [MAVLink packet](https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format).
In order to reduce the amount of information that must be sent, the packet does not include the message metadata, such as what fields are in the message and so on.
Instead, the fields are serialized in a predefined order based on data size and XML definition order, and MAVLink relies on each end of the communication having a shared definition of what messages are being sent.
The shared identity of the message is conveyed by the message id, along with a CRC ("`CRC_EXTRA`") that uniquely identifies the message based on its name and id, and the field names and types.
The receiving end of the communication will discard any packet for which the message id and the `CRC_EXTRA` do not match.
-->

## PX4 and MAVLink

PX4 releases build `common.xml` MAVLink definitions by default, for the greatest compatibility with MAVLink ground stations, libraries, and external components such as MAVLink cameras.
In the `main` branch, these are included from `development.xml` on SITL, and `common.xml` for other boards.

:::info
To be part of a PX4 release, any MAVLink definitions that you use must be in `common.xml` (or included files such as `standard.xml` and `minimal.xml`).
During development you can use definitions in `development.xml`.
You will need to work with the [MAVLink team](https://mavlink.io/en/contributing/contributing.html) to define and contribute these definitions.
:::

PX4 includes the [mavlink/mavlink](https://github.com/mavlink/mavlink) repo as a submodule under [/src/modules/mavlink](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mavlink).
This contains XML definition files in [/mavlink/messages/1.0/](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/).

The build toolchain generates the MAVLink 2 C header files at build time.
The XML file for which headers files are generated may be defined in the [PX4 kconfig board configuration](../hardware/porting_guide_config.md#px4-board-configuration-kconfig) on a per-board basis, using the variable `CONFIG_MAVLINK_DIALECT`:

- For SITL `CONFIG_MAVLINK_DIALECT` is set to `development` in [boards/px4/sitl/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/sitl/default.px4board#L36).
  You can change this to any other definition file, but the file must include `common.xml`.
- For other boards `CONFIG_MAVLINK_DIALECT` is not set by default, and PX4 builds the definitions in `common.xml` (these are build into the [mavlink module](../modules/modules_communication.md#mavlink) by default — search for `menuconfig MAVLINK_DIALECT` in [src/modules/mavlink/Kconfig](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/Kconfig#L10)).

The files are generated into the build directory: `/build/<build target>/mavlink/`.
