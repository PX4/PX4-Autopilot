# uORB Messaging

The Micro Object Request Broker (uORB) is PX4s low-latency asynchronous `publish()` / `subscribe()` messaging API for inter-thread/inter-process communication.
It allows the different modules of the system to communicate effectively, while still being loosely coupled, and hence easily replaced.

## Introduction

PX4 modules communicate using uORB _topics_.
A topic represents a communication channel for sending _messages_ between a publisher module and one or more subscriber modules.
A module that is interested in receiving messages can subscribe to a topic and use it to check for, and read, new messages.
A module that wants to send messages to a particular topic (and hence all subscribers) must _advertise_ that it is going to do so, and can then _publish_ messages when it has new data.

A topic behaves like a queue to which publishers write, and from which subscribers read.
By default he queue can only buffer a single message, which may be overwritten if the publisher writes new message data before subscribers can read it.
The [queue size may be increased](#uorb-buffer-length-orb-queue-length) in the rare cases when subscribers really need to receive every message.

A message is a discrete sample of data that can be published/subscribed via a topic.
The message fields, the constants that can be used with those fields, and the topic(s) to which the message can be published/subscribed are defined in a uORB message definition file.

By default a single topic is automatically created for each message definition file, which is created by `underscore_snake_casing` the (CamelCase) message definition file name.
For example, topic `battery_status` is automatically created for the `BatteryStatus.msg`.
This is generally what you want if the message is always about the same kind of data (batteries in this case) and so all the subscribers will be interested in the same messages.

Sometimes the same message structure can be used to represent data from different kinds of sources, which will have different sets of interested publishers and subscribers.
In this case the topics need to be explicitly declared.
For example the [VehicleGlobalPosition.msg](../msg_docs/VehicleGlobalPosition.md) can be used for sending messages about global position from an estimator, GNSS, or an external source: the fields are the same, but the source and subscribers of the data may be different.

uORB also provides a mechanism to publish multiple independent instances of the same topic.
This is useful, for example, if the system has several sensors of the same type.

uORB is implemented in the [`uorb` module](../modules/modules_communication.md#uorb).
The module is started automatically (with `uorb start`) early in the PX4 boot sequence, as many applications depend on it.
Unit tests can be started with `uorb_tests`.

This document explains how to add uORB message definitions and their corresponding topic(s), how to use reference a topic in code, and how to view topics as they change in PX4.
The [First Application Tutorial (Hello Sky)](../modules/hello_sky.md) provides more comprehensive instructions for how to use topics in C++.

## Adding a New Topic

New uORB topics can be added either within the main PX4/PX4-Autopilot repository, or can be added in an [out-of-tree message definition](../advanced/out_of_tree_modules.md#out-of-tree-uorb-message-definitions).

To add new topics, you need to create a new **.msg** "message definition file" named following the CamelCase convention.
The file should be added to the [msg/](https://github.com/PX4/PX4-Autopilot/tree/main/msg/) directory (or [msg/versioned](https://github.com/PX4/PX4-Autopilot/tree/main/msg/versioned) if it needs to be versioned) and then listed in the `msg/CMakeLists.txt` file.

::: tip
Messages need to be versioned if they are exposed to ROS 2 and needs to remain compatible across multiple ROS and PX4 versions.
See [Message Versioning](#message-versioning) for more information.
:::

A message definition file can define one or more _topics_, which all have the same fields and structure.
By default a definition maps to a single topic that is named using a snake_case version of the message definition file name (for example, `TopicName.msg` would define a topic `topic_name`).
You can also specify multiple topics to be created by the message definition, which is useful when you need several topics that have the same fields and structure (see [Multi-Topic Messages](#multi-topic-messages) below).

The section [Message Definitions](#message-definitions) below describes the message format.

From the message definitions, the needed C/C++ code is automatically generated.

To use the topic in the code, first include the generated header, which will be named using the snake_case version of the (CamelCase) message definition file name.
For example, for a message named `VelocityLimits` you would include `velocity_limits.h` as shown:

```cpp
#include <uORB/topics/velocity_limits.h>
```

In code you refer to the topic using its id, which in this example would be: `ORB_ID(velocity_limits)`.

## Message Definitions

Every message should start with a [message description](#message-description) that outlines its purpose (a comment starts with the `#` symbol and goes to the end of the line).
The message will then define one or more fields, which are defined with a _type_, such as `bool`, `uint8`, and `float32`, followed by a _name_.
By convention, each field is followed by a descriptive _comment_, which is any text from the `#` symbol to the end of the line.

::: info
All _versioned_ messages definitions must include the `uint32 MESSAGE_VERSION` field.
For more information, refer to the [Message Versioning](#message-versioning) section.
:::

::: warning
All message definitions **must** include the `uint64_t timestamp` field, and this should be filled in when publishing the associated topic(s).
This field is needed in order for the logger to be able to record UORB topics.
:::

For example the [VelocityLimits](../msg_docs/VelocityLimits.md) message definition shown below has a descriptive comment, followed by a number of fields, which each have a comment.

```text
# Velocity and yaw rate limits for a multicopter position slow mode only

uint64 timestamp # [us] Time since system start.

# absolute speeds, NAN means use default limit
float32 horizontal_velocity # [m/s] Horizontal velocity.
float32 vertical_velocity # [m/s] Vertical velocity.
float32 yaw_rate # [rad/s] Yaw rate.
```

By default this message definition will be compiled to a single topic with an id `velocity_limits`, a direct conversion from the CamelCase name to a snake_case version.

This is the simplest form of a message.
See the existing [`msg`](../msg_docs/index.md) files for other examples of how messages are defined.

#### Message Description

Every message should start with a [comment](#comments) block that describes the message (one or more of lines that all start with `#`).
The first comment line is mandatory and provides the short description.
This may be followed by an empty comment line and then a optional long description.

```text
# Short description
#
# Long(er) description for the message
# that can be multiline
```

The short description should provide a succinct explanation for the purpose of the message.
Minimally it may just mirror the message name.
For example, [`BatteryStatus`](../msg_docs/BatteryStatus.md) has the short description `Battery status`.

The long description should provide any additional context required to understand what how the message is used.
It might explain who are the publishers and who are the expected consumers, such as MAVLink or the logging system.
It might also cover whether the message is only used for a particular frame type or mode.

Both short and long descriptions may be multi-line.
The long description may also include empty comment lines (the short description cannot, because the first empty comment delineates the short and long description).
A terminating full stop can be omitted from single line comments and from the final line.

The block ends at the first non-comment line, such as an empty line, field, or constant.
Any subsequent comment lines are considered "internal comments".

### Comments

Comments are text provided for explanation or documentation purposes.
Any text after a `#` character is a comment (with the exception of lines that [start with `# TOPIC`](#multi-topic-message)).

PX4 uses structured comments for message, field, and constant descriptions.
Other comments are internal.

### Fields

Messages define one or more fields, which are the variables that are written and read by publishers and subscribers, respectively.

A typical field might look like this:

```sh
float64 lat # [deg] Latitude (WGS84).
```

Each field has a _type_ followed by a _name_, and should also have a _comment_
The format is as shown:

```sh
<type> <name> # [metadata] <description>
```

- `type`:
  - A primitive data type: `bool`, `char`, `uint8`, `uint32`, `uint64`, `int8`, `int16`, `int32`, `float32` and `float64`.
  - Another UORB message name, when creating complex types using [nested messages](#nested-messages).
- `name`
  - Any message-unique string.
    By convention use lower case `underline_snake_case`.

The comment must all be on the same line as the field, and should consist of optional metadata and a description:

- `metadata` (Optional)
  - Information about the field units and allowed values:
    - `[<unit>]`
      - The unit of measurement inside square brackets (note, no `@` delineator indicates a unit).
        For example `[m]` or `[deg]`.
        Typical units include: `m`, `deg`, `m/s`, `rad`, `rad/s`, and so on.
    - `[@enum <enum_name>]`
      - The `enum_name` gives the prefix of constant values in the message that can be assigned to the field.
        Note that in UORB "enums" are a naming convention: they are not explicitly declared.
        Multiple enum names indicate a possible error in the field design.
    - `[@range <lower_value>, <upper_value>]`
      - The allowed range of the field, specified as a `lower_value` and/or an `upper_value`.
        Either value can be omitted to indicate an unbounded upper or lower value.
        For example `[@range 0, 3]`, `[@range 5.3, ]`, `[@range , 3]`.
    - `[@invalid <value> <description>]`
      - The `value` to set the field to indicate that the field doesn't contain valid data.
        The `description` is optional, and might be used to indicate the conditions under which data is invalid.
- `description`
  - A concise description of the purpose of the field and allowed values, and including any important information that can't be inferred from the name!
    Use a capital first letter, and omit the full stop if the description is a single sentence.
    Multiple sentences may also omit the final full stop.

#### Array Fields

A array field defines multiple variables in an array, where all values have the same type.
The number of elements are given using square brackets after the type.
Array fields are otherwise defined (and documented) in the same way as other fields.

For example:

```sh
int32[12] raw_data # ADC channel raw value, accept negative value.
```

#### Mandatory Fields

All message definitions **must** include following fields:

- `uint64_t timestamp`
  - This should be filled in when publishing the associated topic(s).
    It is needed in order for the logger to be able to record UORB topics.
  - The comment should be `# [us] Time since system start`.

### Constants

Constants specify a mapping between a name and a value.

These are mainly used to predefine useful values that you might need to use for a particular field, such as a state or flag values.
Often these are grouped together as [enums](#enums).
There are also a small number of [metadata constants](#metadata-constants) that are used by the build infrastructure.

For example, here are a number of constants for indicating battery warnings.

```sh
uint8 WARNING_NONE = 0 # No battery low voltage warning active
uint8 WARNING_LOW = 1 # Low voltage warning
```

Constants are specified as a field assigned with a value.
The field part has a _type_, which must match the type of the field they are to be used with, followed by a _name_.
They should also have a _comment_ with a description.
By convention they are defined immediately below the field with which they can used.

The format is as shown:

```sh
<type> <name> = <value> # <description>
```

- `type`:
  - Must match the `type` of the field with which it is to be used.
- `name`
  - The name of the constant.
    This must be message-unique and is by convention `ALL_UPPER_CASE_UNDERLINE_SNAKE_CASE`
  - Constant names that can be used with a field should share the same prefix and should indicate the value's purpose.

The comment must all be on the same line as the field.
Note that this is much like the field description (but there is no metadata):

- `description`
  - A terse description of the purpose of the constant.
    Use a capital first letter, and omit the full stop if the description is a single sentence.
    Multiple sentences may also omit the final full stop.

#### Enums

Enums are groups/sets of enumerated constants that can be used as values for a particular field.

UORB does not define a formal syntax for enums.
Instead we use a prefix naming convention to indicate all the constants that are part of the same enum.
The constants in the enum should be declared immediately after the field in which they are used, and for parsing convenience, the prefix is listed in the field using `@enum` metadata.

For example, here is the definition of the `warning` field and some of the `WARNING` enum values that can be used with it:

```sh
uint8 warning # [@enum WARNING] Current battery warning
uint8 WARNING_NONE = 0 # No battery low voltage warning active
uint8 WARNING_LOW = 1 # Low voltage warning
uint8 WARNING_CRITICAL = 2 # Critical voltage, return / abort immediately
...
```

#### Metadata Constants

A number of constants provide information that is used by the PX4 build system to configure how the message may be used, such as the version and length of the message.
If relevant, these should appear near the top of the file, immediately after the [Message Description](#message-description).

The allowed constants are:

- `ORB_QUEUE_LENGTH` - Sets the [uORB Buffer Length](#uorb-buffer-length-orb-queue-length), which is used in rare cases where a subscriber needs all values that are set for a field, rather than just the most recent sample.
- `MESSAGE_VERSION` - Sets the version number of a versioned message.
  This is used as part of the infrastructure to maintain compatibility between PX4 and ROS 2 versions compiled against different message definitions.
  For more information see [Message Versioning](#message-versioning)

#### Multi-Topic Messages (`# TOPICS`) {#multi-topic-messages}

By default a single topic is automatically created for each message definition file, which is created by `underscore_snake_casing` the (CamelCase) message definition file name.
For example, topic `battery_status` is automatically created for the `BatteryStatus.msg`.
This is generally what you want if the message is always about the same kind of data (batteries in this case) and so all the subscribers will be interested in the same messages.

Sometimes it is useful to use the same message definition for multiple topics.
In this case the topics need to be explicitly declared.
You can do this by adding one or more lines to the end of the message prefixed with `# TOPICS`, followed by space-separated topic ids.

For example, the [VehicleGlobalPosition.msg](../msg_docs/VehicleGlobalPosition.md) message definition is used to define the topic ids as shown:

```text
# TOPICS vehicle_global_position vehicle_global_position_groundtruth external_ins_global_position
# TOPICS estimator_global_position
# TOPICS aux_global_position
```

Note that multiple topics are useful in this case because the likely subscribers for the different sources of global position are likely to be different.

### Nested Messages

Message definitions can be nested within other messages to create complex data structures.

To nest a message, simply include the nested message type in the parent message definition. For example, [`PositionSetpoint.msg`](../msg_docs/PositionSetpoint.md) is used as a nested message in the [`PositionSetpointTriplet.msg`](../msg_docs/PositionSetpointTriplet.md) topic message definition.

```text
# Global position setpoint triplet in WGS84 coordinates.
#
# This are the three next waypoints (or just the next two or one).

uint64 timestamp # [us] Time since system start

PositionSetpoint previous
PositionSetpoint current
PositionSetpoint next
```

### uORB Buffer Length (ORB_QUEUE_LENGTH)

uORB messages have a single-message buffer by default, which may be overwritten if the message publication rate is too high.
In most cases this does not matter: either we are only interested in the latest sample of a topic, such as a sensor value or a setpoint, or losing a few samples is not a particular problem.
For relatively few cases, such as vehicle commands, it is important that we don't drop topics.

In order to reduce the chance that messages will be dropped we can use named constant `ORB_QUEUE_LENGTH` to create a buffer of the specified length.
For example, to create a four-message queue, add the following line to your message definition:

```sh
uint8 ORB_QUEUE_LENGTH = 4
```

As long as subscribers are able to read messages out of the buffer quickly enough than it isn't ever fully filled to the queue length (by publishers), they will be able to get all messages that are sent.
Messages will still be lost they are published when the queue is filled.

Note that the queue length value must be a power of 2 (so 2, 4, 8, ...).

### Message/Field Deprecation {#deprecation}

As there are external tools using uORB messages from log files, such as [Flight Review](https://github.com/PX4/flight_review), certain aspects need to be considered when updating existing messages:

- Changing existing fields or messages that external tools rely on is generally acceptable if there are good reasons for the update.
  In particular for breaking changes to _Flight Review_, _Flight Review_ must be updated before code is merged to `master`.
- In order for external tools to reliably distinguish between two message versions, the following steps must be followed:
  - Removed or renamed messages must be added to the `deprecated_msgs` list in [msg/CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/c5a6a60903455c3600f47e3c45ecaa48614559c8/msg/CMakeLists.txt#L189) and the **.msg** file needs to be deleted.
  - Removed or renamed fields must be commented and marked as deprecated.
    For example `uint8 quat_reset_counter` would become `# DEPRECATED: uint8 quat_reset_counter`.
    This is to ensure that removed fields (or messages) are not re-added in future.
  - In case of a semantic change (e.g. the unit changes from degrees to radians), the field must be renamed as well and the previous one marked as deprecated as above.

## Message Versioning (MESSAGE_VERSION) {#message-versioning}

<Badge type="tip" text="PX4 v1.16" />

Optional message versioning was introduced PX4 v1.16 to make it easier to maintain compatibility between PX4 and ROS 2 versions compiled against different message definitions.
Versioned messages are designed to remain more stable over time compared to their non-versioned counterparts, as they are intended to be used across multiple releases of PX4 and external systems, ensuring greater compatibility over longer periods.

Versioned messages include an additional field `uint32 MESSAGE_VERSION = x`, where `x` corresponds to the current version of the message.

Versioned and non-versioned messages are separated in the file system:

- Non-versioned topic message files and [server service](../ros2/user_guide.md#px4-ros-2-service-servers) message files remain in the [`msg/`](https://github.com/PX4/PX4-Autopilot/tree/main/msg) and [`srv/`](https://github.com/PX4/PX4-Autopilot/tree/main/srv) directories, respectively.
- The current (highest) version of message files are located in the `versioned` subfolders ([`msg/versioned`](https://github.com/PX4/PX4-Autopilot/tree/main/msg/versioned) and [`srv/versioned`](https://github.com/PX4/PX4-Autopilot/tree/main/srv/versioned)).
- Older versions of messages are stored in nested `msg/px4_msgs_old/` subfolders ([`msg/px4_msgs_old/msg/`](https://github.com/PX4/PX4-Autopilot/tree/main/msg/px4_msgs_old/msg) and [`msg/px4_msgs_old/srv/`](https://github.com/PX4/PX4-Autopilot/tree/main/msg/px4_msgs_old/srv)).
  The files are also renamed with a suffix to indicate their version number.

::: tip
The file structure is outlined in more detail in [File structure (ROS 2 Message Translation Node)](../ros2/px4_ros2_msg_translation_node.md#file-structure).
:::

The [ROS 2 Message Translation Node](../ros2/px4_ros2_msg_translation_node.md) uses the above message definitions to seamlessly convert messages sent between PX4 and ROS 2 applications that have been compiled against different message versions.

Updating a versioned message involves more steps compared to updating a non-versioned one.
For more information see [Updating a Versioned Message](../ros2/px4_ros2_msg_translation_node.md#updating-a-versioned-message).

For the full list of versioned and non-versioned messages see: [uORB Message Reference](../msg_docs/index.md).

For more on PX4 and ROS 2 communication, see [PX4-ROS 2 Bridge](../ros/ros2_comm.md).

::: info
ROS 2 plans to natively support message versioning in the future, but this is not implememented yet.
See the related ROS Enhancement Proposal ([REP 2011](https://github.com/ros-infrastructure/rep/pull/358)).
See also this [Foxglove post](https://foxglove.dev/blog/sending-ros2-message-types-over-the-wire) on message hashing and type fetching.
:::

## Publishing

Publishing a topic can be done from anywhere in the system, including interrupt context (functions called by the `hrt_call` API).
However, the topic needs to be advertised and published outside of an interrupt context (at least once) before it can be published in an interrupt context.

### Multi-instance

uORB provides a mechanism to publish multiple independent instances of the _same_ topic.
This is useful, for example, if the system has several sensors of the same type.

::: info
This differs from [Multi-Topic Messages](#multi-topic-messages), where we create different topics that happen to have the same structure.
:::

A publisher can call `orb_advertise_multi` to create a new topic instance and get its instance index.
A subscriber will then have to choose to which instance to subscribe to using `orb_subscribe_multi` (`orb_subscribe` subscribes to the first instance).

Make sure not to mix `orb_advertise_multi` and `orb_advertise` for the same topic!

The full API is documented in [platforms/common/uORB/uORBManager.hpp](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/uORB/uORBManager.hpp).

## Listing Topics and Listening in

::: info
The `listener` command available on most boards after FMUv4.
You can check for a particular board by searching for the `CONFIG_SYSTEMCMDS_TOPIC_LISTENER` key in the [kconfig](../hardware/porting_guide_config.md) board configuration (for example, see the FMUv6 [default.px4board](https://github.com/PX4/PX4-Autopilot/blob/release/1.15/boards/px4/fmu-v6x/default.px4board#L100) file).
:::

To list all topics, list the file handles:

```sh
ls /obj
```

To listen to the content of one topic for 5 messages, run the listener:

```sh
listener sensor_accel 5
```

The output is n-times the content of the topic:

```sh
TOPIC: sensor_accel #3
timestamp: 84978861
integral_dt: 4044
error_count: 0
x: -1
y: 2
z: 100
x_integral: -0
y_integral: 0
z_integral: 0
temperature: 46
range_m_s2: 78
scaling: 0

TOPIC: sensor_accel #4
timestamp: 85010833
integral_dt: 3980
error_count: 0
x: -1
y: 2
z: 100
x_integral: -0
y_integral: 0
z_integral: 0
temperature: 46
range_m_s2: 78
scaling: 0
```

:::tip
On NuttX-based systems (Pixhawk, Pixracer, etc) the `listener` command can be called from within the _QGroundControl_ MAVLink Console to inspect the values of sensors and other topics.
This is a powerful debugging tool because it can be used even when QGC is connected over a wireless link (e.g. when the vehicle is flying).
For more information see: [Sensor/Topic Debugging](../debug/sensor_uorb_topic_debugging.md).
:::

### uorb top Command

The command `uorb top` shows the publishing frequency of each topic in real-time:

```sh
update: 1s, num topics: 77
TOPIC NAME                        INST #SUB #MSG #LOST #QSIZE
actuator_armed                       0    6    4     0 1
actuator_controls_0                  0    7  242  1044 1
battery_status                       0    6  500  2694 1
commander_state                      0    1   98    89 1
control_state                        0    4  242   433 1
ekf2_innovations                     0    1  242   223 1
ekf2_timestamps                      0    1  242    23 1
estimator_status                     0    3  242   488 1
mc_att_ctrl_status                   0    0  242     0 1
sensor_accel                         0    1  242     0 1
sensor_accel                         1    1  249    43 1
sensor_baro                          0    1   42     0 1
sensor_combined                      0    6  242   636 1
```

The columns are: topic name, multi-instance index, number of subscribers, publishing frequency in Hz, number of lost messages per second (for all subscribers combined), and queue size.

## Plotting Changes in Topics

Topic changes can be plotted in realtime using PlotJuggler and the PX4 ROS 2 integration (note that this actually plots ROS topics that correspond to uORB topics, but the effect is the same).

For more information see: [Plotting uORB Topic Data in Real Time using PlotJuggler](../debug/plotting_realtime_uorb_data.md).

<video src="../../assets/debug/realtime_debugging/realtime_debugging.mp4" width="720" controls></video>

## See Also

- _PX4 uORB Explained_ Blog series
  - [Part 1](https://px4.io/px4-uorb-explained-part-1/)
  - [Part 2](https://px4.io/px4-uorb-explained-part-2/)
  - [Part 3 (The deep stuff)](https://px4.io/px4-uorb-explained-part-3-the-deep-stuff/)
