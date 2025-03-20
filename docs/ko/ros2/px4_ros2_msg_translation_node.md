# PX4 ROS 2 Message Translation Node

<Badge type="tip" text="main (planned for: PX4 v1.16+)" /> <Badge type="warning" text="Experimental" />

The message translation node allows ROS 2 applications that were compiled against different versions of the PX4 messages to interwork with newer versions of PX4, and vice versa, without having to change either the application or the PX4 side.

## 개요

The translation of messages from one definition version to another is possible thanks to the introduction of [message versioning](../middleware/uorb.md#message-versioning).

The translation node has access to all message versions previously defined by PX4.
It dynamically observes the DDS data space, monitoring the publications, subscriptions and services originating from either PX4 via the [uXRCE-DDS Bridge](../middleware/uxrce_dds.md), or ROS 2 applications.
When necessary, it converts messages to the current versions expected by both applications and PX4, ensuring compatibility.

![Overview ROS 2 Message Translation Node](../../assets/middleware/ros2/px4_ros2_interface_lib/translation_node.svg)

<!-- doc source: ../../assets/middleware/ros2/px4_ros2_interface_lib/translation_node.drawio -->

To support the coexistence of different versions of the same messages within the ROS 2 domain, the ROS 2 topic-names for publications, subscriptions, and services include their respective message version as a suffix.
This naming convention takes the form `<topic_name>_v<version>`, as shown in the diagram above.

## 사용법

### 설치

The following steps describe how to install and run the translation node on your machine.

1. (Optional) Create a new ROS 2 workspace in which to build the message translation node and its dependencies:

  ```sh
  mkdir -p /path/to/ros_ws/src
  ```

2. Run the following helper script to copy the message definitions and translation node into your ROS workspace directory.

  ```sh
  cd /path/to/ros_ws
  /path/to/PX4-Autopilot/Tools/copy_to_ros_ws.sh .
  ```

3. Build and source the workspace.

  ```sh
  colcon build
  source /path/to/ros_ws/install/setup.bash
  ```

4. Finally, run the translation node.

  ```sh
  ros2 run translation_node translation_node_bin
  ```

  You should see an output similar to:

  ```sh
  [INFO] [1734525720.729530513] [translation_node]: Registered pub/sub topics and versions:
  [INFO] [1734525720.729594413] [translation_node]: Registered services and versions:
  ```

With the translation node running, any simultaneously running ROS 2 application designed to communicate with PX4 can do so, as long as it uses message versions recognized by the node.
The translation node will print a warning if it encounters an unknown topic version.

:::info
After making a modification in PX4 to the message definitions and/or translation node code, you will need to rerun the steps above from point 2 to update your ROS workspace accordingly.
:::

### In ROS Applications

While developing a ROS 2 application that communicates with PX4, it is not necessary to know the specific version of a message being used.
The message version can be added generically to a topic name like this:

:::: tabs

:::tab C++

```c++
topic_name + "_v" + std::to_string(T::MESSAGE_VERSION)
```

:::

:::tab Python

```python
topic_name + "_v" + VehicleAttitude.MESSAGE_VERSION
```

:::

::::

where `T` is the message type, e.g. `px4_msgs::msg::VehicleAttitude`.

For example, the following implements a minimal subscriber and publisher node that uses two versioned PX4 messages and topics:

:::: tabs

:::tab C++

```c++
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

// Template function to get the message version suffix
// The correct message version is directly inferred from the message defintion
template <typename T>
std::string getMessageNameVersion() {
    if (T::MESSAGE_VERSION == 0) return "";
    return "_v" + std::to_string(T::MESSAGE_VERSION);
}

class MinimalPubSub : public rclcpp::Node {
  public:
    MinimalPubSub() : Node("minimal_pub_sub") {
      // Use template function to define the correct topics automatically
      const std::string sub_topic = "/fmu/out/vehicle_attitude" + getMessageNameVersion<px4_msgs::msg::VehicleAttitude>();
      const std::string pub_topic = "/fmu/in/vehicle_command" + getMessageNameVersion<px4_msgs::msg::VehicleCommand>();

      _subscription = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
          sub_topic, 10,
          std::bind(&MinimalPubSub::attitude_callback, this, std::placeholders::_1));
      _publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>(pub_topic, 10);
    }

  private:
    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received attitude message.");
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _publisher;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr _subscription;
};
```

:::

:::tab Python

```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleAttitude

# Helper function to get the message version suffix
# The correct message version is directly inferred from the message definition
def get_message_name_version(msg_class):
    if msg_class.MESSAGE_VERSION == 0:
        return ""
    return f"_v{msg_class.MESSAGE_VERSION}"

class MinimalPubSub(Node):
    def __init__(self):
        super().__init__('minimal_pub_sub')

        # Use helper function to define the correct topics automatically
        sub_topic = f"/fmu/out/vehicle_attitude{get_message_name_version(VehicleAttitude)}"
        pub_topic = f"/fmu/in/vehicle_command{get_message_name_version(VehicleCommand)}"

        self._subscription = self.create_subscription(
            VehicleAttitude,
            sub_topic,
            self.attitude_callback,
            10
        )

        self._publisher = self.create_publisher(
            VehicleCommand,
            pub_topic,
            10
        )

    def attitude_callback(self, msg):
        self.get_logger().info("Received attitude message.")
```

:::

::::

On the PX4 side, the DDS client automatically adds the version suffix if a message definition contains the field `uint32 MESSAGE_VERSION = x`.

:::info
Version 0 of a topic means that no `_v<version>` suffix should be added.
:::

## Development

### Definitions

A **message** defines the data format used for communication, whether over a topic or a service.
Therefore a message can be either a _topic_ message defined by a `.msg` file, or a _service_ message defined by a `.srv` file.

A **versioned message** is a message for which changes are tracked and each change results in a version bump, with the previous state of the definition being stored in history.
The latest version of every message is stored in `msg/versioned/` for topics (or `srv/versioned` for services), and all older versions are stored in `msg/px4_msgs_old/msg/` (or `msg/px4_msgs_old/srv/`).

A **version translation** defines a bidirectional mapping of the contents of one or more message definition across different versions.
Each translation is stored as a separate `.h` header file under `msg/translation_node/translations/`.
Message translations can be either _direct_ or _generic_.

- A **direct translation** defines a bidirectional mapping of the contents of a _single_ message between two of its versions.
  This is the simpler case and should be preferred if possible.
- A **generic translation** defines a bidirectional mapping of the contents of `n` input messages to `m` output messages across different versions.
  This can be used for merging or splitting a message, or when moving a field from one message to another.

### File Structure

Starting from PX4 v1.16 (main), the PX4-Autopilot `msg/` and `srv/` directories are structured as follows:

```
PX4-Autopilot
├── ...
├── msg/
  ├── *.msg              # Non-versioned topic message files
  ├── versioned/         # Latest versioned topic message files
  ├── px4_msgs_old/      # History of versioned messages (.msg + .srv) [ROS 2 package]
  └── translation_node/  # Translation node and translation headers [ROS 2 package]
└── srv/
  ├── *.srv              # Non-versioned service message files
  └── versioned/         # Latest versioned service message files
```

This structure introduces new directories: `versioned/`, `px4_msgs_old/`, and `translation_node/`.

#### Directories `msg/versioned/` and `srv/versioned/`

- Contain the current latest version of each message.
- Files in these directories must include a `MESSAGE_VERSION` field to indicate that they are versioned.
- File names follow the conventional naming scheme (without a version suffix).

Example directory structure:

```
PX4-Autopilot
├── ...
├── msg/
  └── versioned/
    ├── VehicleAttitude.msg        # e.g. MESSAGE_VERSION = 3
    └── VehicleGlobalPosition.msg  # e.g. MESSAGE_VERSION = 2
└── srv/
  └── versioned/
    └── VehicleCommand.srv         # e.g. MESSAGE_VERSION = 2
```

#### Directory `px4_msgs_old/`

- Archives the history of all versioned messages, including both topic and service messages (resp. under `msg/` and `srv/` subdirectories).
- Each file includes a `MESSAGE_VERSION` field.
- File names reflect the message's version with a suffix (e.g., `V1`, `V2`).

Example directory structure (matching the example above):

```
  ...
  msg/
  └── px4_msgs_old/
    ├── msg/
      ├── VehicleAttitudeV1.msg
      ├── VehicleAttitudeV2.msg
      └── VehicleGlobalPositionV1.msg
    └── srv/
      └── VehicleCommandV1.srv
```

#### Directory `translation_node/`

- Contains headers for translating between all different versions of messages.
- Each translation (direct or generic) is a single `.h` header file.
- The header `all_translation.h` acts as the main header, and includes all subsequent translation headers.

Example directory structure (matching the example above):

```
  ...
  msg/
  └── translation_node/
    └── translations/
      ├── all_translations.h                        # Main header
      ├── translation_vehicle_attitude_v1.h         # Direct translation v0 <-> v1
      ├── translation_vehicle_attitude_v2.h         # Direct translation v1 <-> v2
      ├── translation_vehicle_attitude_v3.h         # Direct translation v2 <-> latest (v3)
      ├── translation_vehicle_global_position_v1.h  # Direct translation v0 <-> v1
      ├── translation_vehicle_global_position_v2.h  # Direct translation v1 <-> latest (v2)
      ├── translation_vehicle_command_v1.h          # Direct translation v0 <-> v1
      └── translation_vehicle_command_v2.h          # Direct translation v1 <-> latest (v2)
```

### Updating a Versioned Message

This section provides a step-by-step walkthrough and a basic working example of what the process of changing a versioned message looks like.

The example describes the process of updating the `VehicleAttitude` message definition to contain an additional `new_field` entry, incrementing the message version from `3` to `4`, and creating a new direct translation in the process.

1. **Create an Archived Definition of the Current Versioned Message**

  Copy the versioned `.msg` topic message file (or `.srv` service message file) to `px4_msgs_old/msg/` (or `px4_msgs_old/srv/`), and append its message version to the file name.

  For example:<br>
  Copy `msg/versioned/VehicleAttitude.msg` → `msg/versioned/px4_msgs_old/msg/VehicleAttitudeV3.msg`

2. **Update Translation References to the Archived Definition**

  Update the existing translations header files `msg/translation_node/translations/*.h` to reference the newly archived message definition.

  For example, update references in those files:<br>

  - Replace `px4_msgs::msg::VehicleAttitude` → `px4_msgs_old::msg::VehicleAttitudeV3`
  - Replace `#include <px4_msgs/msg/vehicle_attitude.hpp>` → `#include <px4_msgs_old/msg/vehicle_attitude_v3.hpp>`

3. **Update the Versioned Definition**

  Update the versioned `.msg` topic message file (or `.srv` service message file) with required changes.

  First increment the `MESSAGE_VERSION` field.
  Then update the message fields that prompted the version change.

  For example, update `msg/versioned/VehicleAttitude.msg` from:

  ```txt
  uint32 MESSAGE_VERSION = 3
  uint64 timestamp
  ...
  ```

  to

  ```txt
  uint32 MESSAGE_VERSION = 4  # Increment
  uint64 timestamp
  float32 new_field           # Make definition changes
  ...
  ```

4. **Add a New Translation Header**

  Add a new version translation to bridge the archived version and the updated current version, by creating a new translation header.

  For example, create a direct translation header `translation_node/translations/translation_vehicle_attitude_v4.h`:

  ```c++
  // Translate VehicleAttitude v3 <--> v4
  #include <px4_msgs_old/msg/vehicle_attitude_v3.hpp>
  #include <px4_msgs/msg/vehicle_attitude.hpp>

  class VehicleAttitudeV4Translation {
  public:
    using MessageOlder = px4_msgs_old::msg::VehicleAttitudeV3;
    static_assert(MessageOlder::MESSAGE_VERSION == 3);

    using MessageNewer = px4_msgs::msg::VehicleAttitude;
    static_assert(MessageNewer::MESSAGE_VERSION == 4);

    static constexpr const char* kTopic = "fmu/out/vehicle_attitude";

    static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
      msg_newer.timestamp = msg_older.timestamp;
      msg_newer.timestamp_sample = msg_older.timestamp_sample;
      msg_newer.q[0] = msg_older.q[0];
      msg_newer.q[1] = msg_older.q[1];
      msg_newer.q[2] = msg_older.q[2];
      msg_newer.q[3] = msg_older.q[3];
      msg_newer.delta_q_reset = msg_older.delta_q_reset;
      msg_newer.quat_reset_counter = msg_older.quat_reset_counter;

      // Populate `new_field` with some value
      msg_newer.new_field = -1;
    }

    static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
      msg_older.timestamp = msg_newer.timestamp;
      msg_older.timestamp_sample = msg_newer.timestamp_sample;
      msg_older.q[0] = msg_newer.q[0];
      msg_older.q[1] = msg_newer.q[1];
      msg_older.q[2] = msg_newer.q[2];
      msg_older.q[3] = msg_newer.q[3];
      msg_older.delta_q_reset = msg_newer.delta_q_reset;
      msg_older.quat_reset_counter = msg_newer.quat_reset_counter;

      // Discards `new_field` from MessageNewer
    }
  };

  REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleAttitudeV4Translation);
  ```

  Version translation templates are provided here:

  - [Direct Topic Message Translation Template](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/example_translation_direct_v1.h)
  - [Generic Topic Message Translation Template](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/example_translation_multi_v2.h)
  - [Direct Service Message Translation Template](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/example_translation_service_v1.h)

5. **Include New Headers in `all_translations.h`**

  Add all newly created headers to [`translations/all_translations.h`](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/all_translations.h) so that the translation node can find them.

  For example, append the following line to `all_translation.h`:

  ```c++
  #include "translation_vehicle_attitude_v4.h"
  ```

Note that in the example above and in most cases, step 4 only requires the developer to create a direct translation for the definition change.
This is because the changes only involved a single message.
In more complex cases of splitting, merging and/or moving definitions then a generic translation must be created.

For example when moving a field from one message to another, a single generic translation should be added with the two older message versions as input, and the two newer versions as output.
This ensures there is no information lost when translating forward or backward.

This is exactly the approach shown by the [Generic Topic Message Translation Template](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/example_translation_multi_v2.h), omitting only the code for actually modifying fields in the `fromOlder()` and `toOlder()` methods.

:::warning
If a nested message definition changes, all messages including that message also require a version update.
For example this would be the case for message [PositionSetpointTriplet](../msg_docs/PositionSetpointTriplet.md) if it were versioned.
This is primarily important for services which are more likely reference other message definitions.
:::

## Implementation Details

The translation node dynamically monitors the topics and services.
It then instantiates the counterside of the publications and subscribers as required.
For example if there is an external publisher for version 1 of a topic and subscriber for version 2.

Internally, it maintains a graph of all known topic and version tuples (which are the graph nodes).
The graph is connected by the message translations.
As arbitrary message translations can be registered, the graph can have cycles and multiple paths from one node to another.
Therefore on a topic update, the graph is traversed using a shortest path algorithm.
When moving from one node to the next, the message translation method is called with the current topic data.
If a node contains an instantiated publisher (because it previously detected an external subscriber), the data is published.
Thus, multiple subscribers of any version of the topic can be updated with the correct version of the data.

For translations with multiple input topics, the translation continues once all input messages are available.

## 제한 사항

- Translation of service messages does not work on ROS Humble, but does on ROS Jazzy.
  This is because the current implementation depends on a service API that is not yet available in ROS Humble.
  Translation of topic messages is fully supported.
- Services messages only support a linear history, i.e. no message splitting or merging.
- Having both publishers and subscribers for two different versions of the same topic is currently not handled by the translation node and would trigger infinite circular publications.
  This refers to the following problematic configuration:

  ```
  app 1: pub topic_v1, sub topic_v1
  app 2: pub topic_v2, sub topic_v2
  ```

  In practice this configuration is unlikely to occur because ROS topics shared with the FMU are intended to be directional (e.g. `/fmu/out/vehicle_status` or `/fmu/in/trajectory_setpoint`), therefore apps typically do not publish and subscribe simultaneously to the same topic.
  The translation node could be extended to handle this corner-case if required.

Original document with requirements: https://docs.google.com/document/d/18_RxV1eEjt4haaa5QkFZAlIAJNv9w5HED2aUEiG7PVQ/edit?usp=sharing
