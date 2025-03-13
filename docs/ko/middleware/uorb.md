# uORB 메시징

## 소개

The uORB is an asynchronous `publish()` / `subscribe()` messaging API used for inter-thread/inter-process communication.

uORB is implemented in the [`uorb` module](../modules/modules_communication.md#uorb).
It is started automatically (with `uorb start`) early in the PX4 boot sequence, as many applications depend on it.
Unit tests can be started with `uorb_tests`.

This document explains how to add uORB message definitions and their corresponding topic(s), how to use reference a topic in code, and how to view topics as they change in PX4.
The [First Application Tutorial (Hello Sky)](../modules/hello_sky.md) provides more comprehensive instructions for how to use topics in C++.

## Adding a New Topic

New uORB topics can be added either within the main PX4/PX4-Autopilot repository, or can be added in an [out-of-tree message definition](../advanced/out_of_tree_modules.md#out-of-tree-uorb-message-definitions).

To add new topics, you need to create a new **.msg** "message definition file" named following the CamelCase convention.
The file should be added to the [msg/](https://github.com/PX4/PX4-Autopilot/tree/main/msg/) directory (or [msg/versioned](https://github.com/PX4/PX4-Autopilot/tree/main/msg/versioned) if it needs to be versioned) and then listed in the `msg/CMakeLists.txt` file.

:::tip
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

The message definition should start with a descriptive _comment_ that outlines its purpose (a comment starts with the `#` symbol and goes to the end of the line).
The message will then define one or more fields, which are defined with a _type_, such as `bool`, `uint8`, and `float32`, followed by a _name_.
By convention, each field is followed by a descriptive _comment_, which is any text from the `#` symbol to the end of the line.

:::warning
All message definitions **must** include the `uint64_t timestamp` field, and this should be filled in when publishing the associated topic(s).
This field is needed in order for the logger to be able to record UORB topics.
:::

:::info
All _versioned_ messages definitions must include the `uint32 MESSAGE_VERSION` field.
For more information, refer to the [Message Versioning](#message-versioning) section.
:::

For example the [VelocityLimits](../msg_docs/VelocityLimits.md) message definition shown below has a descriptive comment, followed by a number of fields, which each have a comment.

```text
# Velocity and yaw rate limits for a multicopter position slow mode only

uint64 timestamp # time since system start (microseconds)

# absolute speeds, NAN means use default limit
float32 horizontal_velocity # [m/s]
float32 vertical_velocity # [m/s]
float32 yaw_rate # [rad/s]
```

By default this message definition will be compiled to a single topic with an id `velocity_limits`, a direct conversion from the CamelCase name to a snake_case version.

This is the simplest form of a message.
See the existing [`msg`](../msg_docs/index.md) files for other examples of how messages are defined.

### Multi-Topic Messages

Sometimes it is useful to use the same message definition for multiple topics.
This can be specified at the end of the message using a line prefixed with `# TOPICS `, followed by space-separated topic ids.
For example, the [ActuatorOutputs](../msg_docs/ActuatorOutputs.md) message definition is used to define the topic ids as shown:

```text
# TOPICS actuator_outputs actuator_outputs_sim actuator_outputs_debug
```

### Nested Messages

Message definitions can be nested within other messages to create complex data structures.

To nest a message, simply include the nested message type in the parent message definition. For example, [`PositionSetpoint.msg`](../msg_docs/PositionSetpoint.md) is used as a nested message in the [`PositionSetpointTriplet.msg`](../msg_docs/PositionSetpointTriplet.md) topic message definition.

```text
# Global position setpoint triplet in WGS84 coordinates.
# This are the three next waypoints (or just the next two or one).

uint64 timestamp		# time since system start (microseconds)

PositionSetpoint previous
PositionSetpoint current
PositionSetpoint next
```

### Message/Field Deprecation {#deprecation}

As there are external tools using uORB messages from log files, such as [Flight Review](https://github.com/PX4/flight_review), certain aspects need to be considered when updating existing messages:

- 업데이트에 대한 타당한 이유가 있는 경우에는, 외부 도구가 의존하는 기존 필드 또는 메시지를 변경하는 것이 일반적으로 허용됩니다.
  In particular for breaking changes to _Flight Review_, _Flight Review_ must be updated before code is merged to `master`.
- 외부 도구가 두 메시지 버전을 확실하게 구별하려면, 다음 단계를 따라야 합니다.
  - Removed or renamed messages must be added to the `deprecated_msgs` list in [msg/CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/c5a6a60903455c3600f47e3c45ecaa48614559c8/msg/CMakeLists.txt#L189) and the **.msg** file needs to be deleted.
  - 제거되거나 이름이 변경된 필드는 주석을 달고 더 이상 사용되지 않는 것으로 표시하여야 합니다.
    For example `uint8 quat_reset_counter` would become `# DEPRECATED: uint8 quat_reset_counter`.
    이것은 제거된 필드(또는 메시지)가 다시 추가되지 않도록 하기 위한 것입니다.
  - 의미 변경의 경우(예: 단위가 도에서 라디안으로 변경) 필드 이름도 변경하여야 하며, 이전 필드도 위와 같이 더 이상 사용되지 않는 것으로 표시되어야 합니다.

## Message Versioning

<Badge type="tip" text="main (planned for: PX4 v1.16+)" />

Optional message versioning was introduced in the `main` branch (planned for PX4 v1.16+) to make it easier to maintain compatibility between PX4 and ROS 2 versions compiled against different message definitions.
Versioned messages are designed to remain more stable over time compared to their non-versioned counterparts, as they are intended to be used across multiple releases of PX4 and external systems, ensuring greater compatibility over longer periods.

Versioned messages include an additional field `uint32 MESSAGE_VERSION = x`, where `x` corresponds to the current version of the message.

Versioned and non-versioned messages are separated in the file system:

- Non-versioned topic message files and [server service](../ros2/user_guide.md#px4-ros-2-service-servers) message files remain in the [`msg/`](https://github.com/PX4/PX4-Autopilot/tree/main/msg) and [`srv/`](https://github.com/PX4/PX4-Autopilot/tree/main/srv) directories, respectively.
- The current (highest) version of message files are located in the `versioned` subfolders ([`msg/versioned`](https://github.com/PX4/PX4-Autopilot/tree/main/msg/versioned) and [`srv/versioned`](https://github.com/PX4/PX4-Autopilot/tree/main/srv/versioned)).
- Older versions of messages are stored in nested `msg/px4_msgs_old/` subfolders ([`msg/px4_msgs_old/msg/`](https://github.com/PX4/PX4-Autopilot/tree/main/msg/px4_msgs_old/msg) and [`msg/px4_msgs_old/srv/`](https://github.com/PX4/PX4-Autopilot/tree/main/msg/px4_msgs_old/srv)).
  The files are also renamed with a suffix to indicate their version number.

:::tip
The file structure is outlined in more detail in [File structure (ROS 2 Message Translation Node)](../ros2/px4_ros2_msg_translation_node.md#file-structure).
:::

The [ROS 2 Message Translation Node](../ros2/px4_ros2_msg_translation_node.md) uses the above message definitions to seamlessly convert messages sent between PX4 and ROS 2 applications that have been compiled against different message versions.

Updating a versioned message involves more steps compared to updating a non-versioned one.
For more information see [Updating a Versioned Message](../ros2/px4_ros2_msg_translation_node.md#updating-a-versioned-message).

For the full list of versioned and non-versioned messages see: [uORB Message Reference](../msg_docs/index.md).

For more on PX4 and ROS 2 communication, see [PX4-ROS 2 Bridge](../ros/ros2_comm.md).

:::info
ROS 2 plans to natively support message versioning in the future, but this is not implememented yet.
See the related ROS Enhancement Proposal ([REP 2011](https://github.com/ros-infrastructure/rep/pull/358)).
See also this [Foxglove post](https://foxglove.dev/blog/sending-ros2-message-types-over-the-wire) on message hashing and type fetching.
:::

## 퍼블리시(게시)

Publishing a topic can be done from anywhere in the system, including interrupt context (functions called by the `hrt_call` API).
However, the topic needs to be advertised and published outside of an interrupt context (at least once) before it can be published in an interrupt context.

### 다중 인스턴스

uORB provides a mechanism to publish multiple independent instances of the _same_ topic.
This is useful, for example, if the system has several sensors of the same type.

:::info
This differs from [Multi-Topic Messages](#multi-topic-messages), where we create different topics that happen to have the same structure.
:::

A publisher can call `orb_advertise_multi` to create a new topic instance and get its instance index.
A subscriber will then have to choose to which instance to subscribe to using `orb_subscribe_multi` (`orb_subscribe` subscribes to the first instance).

Make sure not to mix `orb_advertise_multi` and `orb_advertise` for the same topic!

The full API is documented in [platforms/common/uORB/uORBManager.hpp](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/uORB/uORBManager.hpp).

## 토픽 나열 및 듣기

:::info
The `listener` command available on most boards after FMUv4.
You can check for a particular board by searching for the `CONFIG_SYSTEMCMDS_TOPIC_LISTENER` key in the [kconfig](../hardware/porting_guide_config.md) board configuration (for example, see the FMUv6 [default.px4board](https://github.com/PX4/PX4-Autopilot/blob/release/1.15/boards/px4/fmu-v6x/default.px4board#L100) file).
:::

모든 토픽을 나열하려면, 파일 핸들을 나열하십시오.

```sh
ls /obj
```

5개의 메시지에 대해 한 토픽의 내용을 들으려면, 리스너를 실행하십시오.

```sh
listener sensor_accel 5
```

출력은 주제 내용의 n배입니다.

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
이것은 QGC가 무선 틍신으로 연결된 경우에도(예: 차량이 비행 중일 때) 사용할 수 있는 강력한 디버깅 도구입니다.
For more information see: [Sensor/Topic Debugging](../debug/sensor_uorb_topic_debugging.md).
:::

### uorb top 명령어

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

각 칼럼은 주제 이름, 다중 인스턴스 인덱스, 구독자 수, 게시 빈도(Hz), 초당 손실된 메시지 수(모든 구독자가 결합된 경우) 및 대기열 크기입니다.

## Plotting Changes in Topics

Topic changes can be plotted in realtime using PlotJuggler and the PX4 ROS 2 integration (note that this actually plots ROS topics that correspond to uORB topics, but the effect is the same).

For more information see: [Plotting uORB Topic Data in Real Time using PlotJuggler](../debug/plotting_realtime_uorb_data.md).

<video src="../../assets/debug/realtime_debugging/realtime_debugging.mp4" width="720" controls></video>

## See Also

- _PX4 uORB Explained_ Blog series
  - [Part 1](https://px4.io/px4-uorb-explained-part-1/)
  - [Part 2](https://px4.io/px4-uorb-explained-part-2/)
  - [Part 3 (The deep stuff)](https://px4.io/px4-uorb-explained-part-3-the-deep-stuff/)
