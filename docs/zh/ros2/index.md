# ROS 2

[ROS 2](https://docs.ros.org/en/humble/#)是一款功能强大的通用机器人开发库，可与 PX4 自动驾驶仪搭配使用，以开发功能丰富的无人机应用程序。

:::warning
小提示
PX4 开发团队强烈建议您使用此 ROS 版本，或将现有系统迁移至此 ROS 版本！

这是最新版本的 [ROS](https://www.ros.org/)(机器人操作系统)。
它在 ROS “1” 的基础上进行了显著改进，尤其能够实现与 PX4 更深度、更低延迟的集成。
:::

ROS的优势在于拥有活跃的开发者生态系统 —— 该生态系统致力于解决各类常见的机器人技术问题，同时还能调用其他为 Linux 系统编写的软件库。
例如，它可以用于 [计算机试图](../computer_vision/index.md) 解决问题。

ROS 2 能够实现与 PX4 极深度的集成，你可以在 ROS 2 中创建飞行模式，这些模式与 PX4 内部原生飞行模式毫无区别；同时还能以高速率直接读取和写入 PX4 内部的 uORB 主题。
（尤其）建议在以下场景中使用：从伴飞计算机进行控制与通信（且低延迟至关重要时）、需借助 Linux 系统的现有库时，或编写新的高级飞行模式时。

ROS 2 与 PX4 之间的通信使用的中间件需实现 [XRCE-DDS protocol](../middleware/uxrce_dds.md).
这个中间件将以 ROS 2 消息和类型显示 PX4  [uORB messages](../msg_docs/index.md) 会转换为 ROS 2 消息和数据类型，从而切实支持从 ROS 2 工作流与节点直接访问 PX4。
中间件使用 uORB 消息定义生成代码来序列化和反序列化来处理PX4 的收发消息。
这些相同的消息定义也用于 ROS 2 应用程序中以便能够解析这些消息。

:::info
ROS 2 也可以使用 [MAVROS](https://github.com/mavlink/mavros/tree/ros2/mavros)而不是 XRCE-DDS连接到 PX4。
该选项受 MAVROS 项目支持（本文档未对此进行说明）。
:::

要通过 XRCE-DDS 有效使用 [ROS 2](../ros2/user_guide.md) ，（在撰写本文时）你必须对 PX4 的内部架构及约定有一定了解，而这些架构与约定和 ROS 所使用的存在差异。
我们计划近期提供ROS 2 API 以对 PX4 的特性进行封装，并举例说明它们的用途。

## Topics

本节的主要主题是：

- [ROS 2 用户指南](../ros2/user_guide.md): PX4 视角下的 ROS 2，包括安装、设置和如何构建与 PX4 通信的 ROS 2 应用。
- [ROS 2 离板控制实例](../ros2/offboard_control.md)：一个 C++ 教程示例显示如何在 [离板模式] (../flight_modes/offboard.md) 中使用 ROS 2 节点进行位置控制。
- [ROS 2 多载具模拟](../ros2/multi_vehicle.md)：通过单独的ROS2 代理商连接到多极PX4 模拟的说明。
- [PX4 ROS2 接口库](../ros2/px4_ros2_interface_lib.md)：一个C++ 库，它与ROS2的 PX4 交互。
  可以使用 ROS 2 创建和注册飞行模式，并从 ROS2 应用程序如VIO 系统发送位置估计数。
- [ROS 2 消息翻译节点](../ros2/px4_ros2_msg_translation_node.md)：一个 ROS 2 消息翻译节点，它允许在 PX4 和 ROS 2 应用程序之间共享，这些应用程序被编译成不同的消息版本。

## 更多信息

- [XRCE-DDS (PX4-ROS 2/DDS Bridge)](../middleware/uxrce_dds.md): PX4 使用中间件链接到 ROS 2。
