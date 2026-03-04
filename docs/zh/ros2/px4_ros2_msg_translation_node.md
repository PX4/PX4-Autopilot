# PX4 ROS 2 消息翻译节点

<Badge type="tip" text="PX4 v1.16" /> <Badge type="warning" text="Experimental" />

消息翻译节点允许针对不同版本的 PX4 消息编译的 ROS 2 应用程序与更新版本的 PX4 交互。 反之亦然，而不必更改应用程序或PX4一侧。

## 综述

由于引入了[消息版本](../middleware/uorb.md#message-versioning)，将消息从一个定义版本翻译成另一个定义版本是可能的。

翻译节点可以访问之前由 PX4 定义的所有消息版本。
它积极观察了光盘系统的数据空间，监测出版物。 通过[uXRCE-DDS桥](../middleware/uxrce_dds.md)或外空委2应用源于PX4的订阅和服务。
必要时，它会将消息转换到应用程序和PX4预期的当前版本，确保兼容性。

！[概述ROS 2消息转换节点]
(../../assets/middleware/ros2/px4_ros2_interface_lib/translation_node.svg)

<!-- doc source: ../../assets/middleware/ros2/px4_ros2_interface_lib/translation_node.drawio -->

支持不同版本的同一消息在交战规则2域内共存， 出版、订阅和服务的ROS 2主题名称包括各自的信息版本的后缀。
这个命名协议的形式为`<topic_name>_v<version>`。

## 用法

### 安装

以下步骤描述如何在您的机器上安装和运行翻译节点。

1. (可选) 创建一个新的 ROS2 工作空间，用于构建消息翻译节点及其依赖：

   ```sh
   mkdir -p /path/to/ros_ws/src
   ```

2. 运行下面的帮助脚本来复制消息定义并将节点翻译到您的ROS工作区目录。

   ```sh
   cd /path/to/ros_ws
   /path/to/PX4-Autopilot/Tools/copy_to_ros_ws.sh .
   ```

3. 构建并源自工作区。

   ```sh
   colcon build
   source /path/to/ros_ws/install/setup.bash
   ```

4. 最后，运行翻译节点。

   ```sh
   ros2 run translation_node translation_node_bin
   ```

   您应该看到一个相似的输出：

   ```sh
   [INFO] [1734525720.729530513] [translation_node]: 注册的 pub/子主题和版本:
   [INFO] [1734525720.729594413] [translation_node]: 注册的服务和版本:
   ```

在正在运行翻译节点时，任何同时运行旨在与 PX4 通信的 ROS 2 应用程序都可以这样做。 只要它使用节点承认的消息版本。
翻译节点如果遇到未知的主题版本，将打印警告。

:::info
在 PX4 修改消息定义和/或翻译节点代码后， 您需要从点2重新运行以上步骤来相应更新您的ROS工作区。
:::

### 在ROS 应用中

开发与 PX4 通信的 ROS 2 应用程序时，不必知道正在使用的消息的特定版本。
消息版本可以以如以下方式一般添加到主题名称中：

:::: tabs

:::tab C++

```c++
主题名称+ "_v" + std::to_string(T::MESSAGE_VERSION)
```

:::

:::tab Python

```python
主题名称 + "_v" + VehicleAttitude.MESSAGE_VERSION
```

:::

::::

其中‘T’消息类型，例如`px4_msgs::msg::VehicleAttitude`.

例如，以下是一个实现最小化订阅者和发布者节点的示例，该节点使用两个带版本的 PX4 消息和主题：

:::: tabs

:::tab C++

```c++
#include <0>
#include <1>
#include <2>
#include <3>

// Template function to get the message version suffix
// The correct message version is directly inferred from the message defintion
template <4>
std::string getMessageNameVersion() {
    if (T::MESSAGE_VERSION == 0) return "";
    return "_v" + std::to_string(T::MESSAGE_VERSION);
}

class MinimalPubSub : public rclcpp::Node {
  public:
    MinimalPubSub() : Node("minimal_pub_sub") {
      // Use template function to define the correct topics automatically
      const std::string sub_topic = "/fmu/out/vehicle_attitude" + getMessageNameVersion<5>();
      const std::string pub_topic = "/fmu/in/vehicle_command" + getMessageNameVersion<6>();

      _subscription = this->create_subscription<5>(
          sub_topic, 10,
          std::bind(&MinimalPubSub::attitude_callback, this, std::placeholders::_1));
      _publisher = this->create_publisher<6>(pub_topic, 10);
    }

  private:
    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received attitude message.");
    }

    rclcpp::Publisher<6>::SharedPtr _publisher;
    rclcpp::Subscription<5>::SharedPtr _subscription;
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

在 PX4 侧，如果消息定义包含字段`uint32 MESSAGE_VERSION = x`，DDS客户端自动添加版本后缀。

:::info
主题版本0意味着不应添加`_v<version>`后缀。
:::

## 开发

### 定义

**消息** 定义了用于通信的数据格式，无论是主题还是服务。
因此，消息可以是 ".msg" 文件定义的 _topic_ 消息，也可以是 ".srv" 文件定义的 _service_ 消息。

**版本信息** 是指跟踪更改的消息，每次变更都会导致版本号递增，且定义的历史状态会被保存下来。
每个消息的最新版本都存储在 `msg/versioned/` 中，用于主题(或`srv/versioned` 用于服务)， 所有旧版本都存储在`msg/px4_msgs_old/msg/`(或`msg/px4_msgs_old/srv/`)中。

**版本翻译**定义了一个或多个消息定义的内容在不同版本之间的双向映射。
每个翻译以单独的 .h 头文件形式存储在 msg/translation_node/translations/ 目录下。
消息翻译可以是 _direct_或 _generic_。

- **直接翻译** 定义一个双向映射其两个版本之间消息的内容。
  这是更简单的情况，在可能的情况下应优先选择。
- **通用翻译** 定义了双向映射`n`输入消息的内容到不同版本的`m`输出消息。
  这可用于合并或拆分消息，也可用于将某个字段从一个消息迁移至另一个消息。

### File Structure

从 PX4 v1.16 版本开始，PX4-Autopilot（PX4 自动驾驶系统）的 msg/ 和 srv/ 目录结构如下：

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

这个结构引入了新的目录：“versioned/`，`px4_msgs_old/`，以及`translation_node/\`。

#### 目录`msg/versioned/` 和 `srv/versioned/`

- 包含每条消息当前的最新版本。
- 这些目录中的文件必须包含 `MESSAGE_VERSION` 字段以表明它们是版本控制。
- 文件名遵循常规命名规则（不带版本后缀）。

示例目录结构：

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

#### 目录`px4_msgs_old/`

- 将所有版本信息的历史记录存档，包括主题和服务信息(resp. under `msg/`和`srv/`子目录)。
- 每个文件都包含一个 MESSAGE_VERSION 字段。
- 文件名反映了带有后缀的消息版本(如：`V1`、`V2`)。

示例目录结构(匹配上面的示例)：

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

#### 目录`translation_node/`

- 包含用于在所有不同版本的消息之间进行翻译的标题。
- 每个翻译 (直接或通用) 都是单个的 `.h` 标题文件。
- 标题`all_translation.h`是主标题，包含所有其后的翻译标题。

示例目录结构(匹配上面的示例)：

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

### 正在更新版本信息...

本节提供了分步操作指南以及一个基础的可运行示例，以展示修改版本化消息的流程具体是怎样的。

该示例描述了更新VehicleAttitude消息定义的流程，具体包括：为其添加一个额外的new_field字段、将消息版本从3递增至4，并在此过程中创建一个新的直接转换。

1. **创建当前版本信息的存档定义**

   将版本号的`.msg`主题消息文件(或`.srv`服务消息文件)复制到`px4_msgs_old/msg/`(或`px4_msgs_old/srv/`)，并将其消息版本附加到文件名。

   例如：<br>
   复制  `msg/versioned/VehicleAttitude.msg` → `msg/versioned/px4_msgs_old/msg/VehicleAttitudeV3.msg`

2. **更新对存档定义的转化引用**

   更新现有翻译标头文件 `msg/translation_node/translations/*.h` 以参考新存档的消息定义。

   例如，更新这些文件中的引用:<br>

   - 替换 `px4_msgs::msg::VehicleAttitude` → `px4_msgs_old::msg::VehicleAttitudeV3`
   - 替换`#include <px4_msgs/msg/vehicle_attitude.hpp>` -> \`#include <px4_msgs_old/msg/vehicle_attitude_v3.hpp>

3. **更新版本定义**

   更新版本的 `.msg` 主题消息文件 (或`.srv` 服务消息文件) 并进行必要的更改。

   第一次递增`MESSAGE_VERSION`字段。
   然后更新促使版本变更的消息字段。

   例如，更新 `msg/versioned/vehicleAttitde.msg` 从：

   ```txt
   uint32 MESSAGE_VERSION = 3
   uint64 timestamp
   ...
   ```

   到

   ```txt
   uint32 MESSAGE_VERSION = 4  # Increment
   uint64 timestamp
   float32 new_field           # Make definition changes
   ```

4. **添加新的翻译标头**

   通过创建一个新的翻译标头来添加一个新的版本翻译来连接存档版本和更新的当前版本。

   例如，创建一个直接翻译标题`translation_node/translation_vaille_attitude_v4.h`：

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
   ```

   版本翻译模板在此提供：

   - [Direct Topic Message Translation Template](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/example_translation_direct_v1.h)
   - [Generic Topic Message Translation Template](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/example_translation_multi_v2.h)
   - [Direct Service Message Translation Template](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/example_translation_service_v1.h)

5. **在`all_translations.h`中包含新标头**

   将所有新创建的标题添加到[`translations/all_translations.h`](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/all_translations.h)，以便翻译节点能够找到它们。

   例如，在`all_translation.h`上加上以下一行：

   ```c++
   #include "translation_vehicle_attitude_v4.h"
   ```

请注意，在上述示例中以及在大多数情况下，步骤 4 仅要求开发者针对此次定义变更创建一个直接转换。
这是因为更改只涉及一个消息。
在更复杂的分割、合并和/或移动定义的情况下，必须创建一个通用转化。

例如，当一个字段从一个消息移动到另一个消息时。 应增加一个单一通用翻译，两个较旧的信息版本作为输入，两个较新的版本作为输出。
这就确保了在向前或向后翻译时不会丢失信息。

这正是 [Generic Topic Message Translation Template](https://github.com/PX4/PX4-Autopilot/blob/main/msg/translation_node/translations/example_translation_multi_v2.h) 所显示的方法。 只省略`fromOlder()` 和 `toOlder()` 方法中的字段的代码。

:::warning
如果嵌套消息定义发生了变化，包括该消息在内的所有消息也需要版本更新。
例如，如果消息 [PositionSetpointTriplet](../msg_docs/PositionSetpointTriplet.md)有版本，将会出现这种情况。
这一点对于服务而言尤为重要，因为服务更有可能引用其他消息定义。
:::

## 实现细节

转换节点动态监测主题和服务。
然后视需要举例说明出版物和订阅者的对应情况。
例如，如果主题第1版有外部发行商，第2版则有订阅商。

在内部，它保存一份所有已知主题和版本管束的图表(即图节点)。
该图通过消息转换实现连接。
由于任意的消息转换可以注册，图可以有周期和从一个节点到另一个节点的多个路径。
因此，在主题更新中，图形使用最短路径算法。
当从一个节点移动到另一个节点时，消息翻译方法将与当前主题数据调用。
如果节点包含实例化发布器 (因为它先前检测到外部订阅者)，数据将被发布。
这样，本专题任何版本的多个订阅者都可以用正确的数据更新数据。

对于有多个输入主题的转化，转化将在所有输入消息都可用后继续。

## 局限

- 服务消息的转化不适用于 ROS Humble，而是适用于ROSJazzy。
  这是因为当前的实现取决于尚未在ROS人类中提供的服务 API 。
  完全支持主题信息转化。
- 服务消息只支持线性历史记录，即没有消息拆分或合并。
- 两个不同版本的同一主题的出版商和订户目前都不是由转化节点处理的，会引发无限的循环出版物。
  这是指下列有问题的配置：

  ```
  app 1: pub topic_v1, sub topic_v1
  app 2: pub topic_v2, sub topic_v2
  ```

  实际上，这种配置不大可能发生，因为与金融监督单位共享的外空系统专题是指向的(例如)。 `/fmu/out/vehicle_status` 或 `/fmu/in/tracjectory_setpoint` ，因此应用通常不会同时发布和订阅相同的主题。
  如果需要，可以扩展翻译节点来处理这个卷轴。

需要原始文件：https://docs.google.com/document/d/18_RxV1eEjt4haaa5QkFZAlIAJNv9w5HED2aUEiG7PVQ/edit?usp=sharing
