# ROS 2

ROS 2 是ROS (机器人操作系统)最新版本 ， 一个通用的机器人库，可以与 PX4 自驾仪一起创建强大的无人机应用。

:::warning
小提示
PX4开发团队强烈建议您使用/迁移到此版本的 ROS！

这是最新版本的 [ROS](https://www.ros.org/) (机器人操作系统)。
它大大改进了ROS1，特别是允许与PX4更深、更低的延迟结合。
:::

ROS得益于一个活跃的生态系统，在这个生态系统里，开发者会解决常见的机器人问题，他们也为Linux编写软件库。
例如，它可以用于[计算机视觉](../computer_vision/index.md)解决方案。

ROS 2深度与 PX4 集成， 在某种程度上，您能够在 ROS 2 中创建与内部 PX4 模式无法区分的飞行模式， 并高效率的直接订阅uORB内部的话题。
建议（尤其是）从同伴计算机进行控制和通信，因为这种计算机的延迟率很低。 当利用来自Linux的现有库时，或在编写新的高层飞行模式时。

ROS 2 和 PX4 之间的通信使用中间件实现[XRCE-DDS] (../middleware/uxrce_dds.md)。
这个中间件将以 ROS 2 消息和类型显示 PX4 [uORB 消息](../msg_docs/index.md)， 有效地允许从 ROS 2 工作流和节点直接访问 PX4。
中间件使用 uORB 消息定义生成代码来序列化和反序列化来处理PX4 的收发消息。
在ROS 2 应用中使用相同的消息定义可直接进行解析。

:::info
ROS 2 也可以使用 [MAVROS](https://github.com/mavlink/mavros/tree/ros2/mavros而不是 XRCE-DDS连接到 PX4。
这一备选办法得到了MAVROS项目的支持(此处没有记录)。
:::

在XRCE-DDS上有效使用[ROS2](../ros2/user_guide.md)。 (撰写本文时)你必须对PX4内部结构和公约有合理的理解，这些结构和公约不同于交战规则所用的内部结构和公约。
我们计划近期提供ROS 2 API 以对 PX4 的特性进行封装，并举例说明它们的用途。

## Topics

本节的主要主题是：

- ROS 2 用户指南: PX4 视角下的 ROS 2，包括安装、设置和如何构建与 PX4 通信的 ROS 2 应用。
- [ROS 2 离板控制示例](../ros2/offboard_control.md)：一个 C++ 教程示例显示如何在 [离板模式] (../flight_modes/offboard.md) 中使用 ROS 2 节点进行位置控制。
- [ROS 2 多车辆模拟](../ros2/multi_vehicle.md)：通过单独的ROS2 代理商连接到多极PX4 模拟的说明。
- [PX4 ROS2 接口库](../ros2/px4_ros2_interface_lib.md)：一个C++ 库，它与ROS2的 PX4 交互。
  可以使用 ROS 2 创建和注册飞行模式，并从 ROS2 应用程序如VIO 系统发送位置估计数。
- [ROS 2 消息翻译节点](../ros2/px4_ros2_msg_translation_node.md)：一个 ROS 2 消息翻译节点，它允许在 PX4 和 ROS 2 应用程序之间共享，这些应用程序被编译成不同的消息版本。

## 更多信息

- [XRCE-DDS (PX4-ROS 2/DDS Bridge)](../middleware/uxrce_dds.md): PX4 使用中间件链接到 ROS 2。
