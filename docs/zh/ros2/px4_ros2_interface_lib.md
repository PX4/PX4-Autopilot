# PX4 ROS 2 接口库

<Badge type="tip" text="PX4 v1.15" /> <Badge type="warning" text="Experimental" />

:::warning
Experimental
在撰写本文时，PX4 ROS 2 接口库的部分内容仍处于试验阶段，因此可能会发生变动。
:::

[PX4 ROS 2 接口库 ]（https://github.com/Auterion/px4-ros2-interface-lib）是一个 C++ 库，可简化从 ROS 2 对 PX4 进行控制和交互的操作。

该库为开发者提供了两个高级接口。

1. [Control Interface](./px4_ros2_control_interface.md) 允许开发者创建并动态注册使用 ROS2 编写的模式。
   它为发送不同类型的设置点提供了课程，涵盖范围从高级导航任务一直到直接执行器控制。
2. [导航界面](./px4_ros2_navigation_interface.md) 允许从ROS 2应用程序（如VIO系统）向PX4发送车辆位置估计数。
3. [Waypoint Missions](./px4_ros2_waypoint_missions.md) 允许航点飞行任务完全在ROS2中运行。

## 在 ROS 2 工作区中安装

要开始使用现有ROS2工作空间内的库：

1. 请确保您在 ROS 2 工作区中有 [ROS 2 设置](../ros2/user_guide.md) 与 [`px4_msgs`](https://github.com/PX4/px4_msgs]。

2. 将代码仓库克隆到工作空间中

   ```sh
   cd $ros_workspace/src
   git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib
   ```

   提示信息
   为确保兼容性，请使用 PX4、px4_msgs（PX4 消息包）及该库的最新 main 分支。
   另请参阅 [here]（https://github.com/Auterion/px4-ros2-interface-lib#compatibility-with-px4）

:::

3. 构建工作空间:

   ```sh
   cd ..
   colcon building
   source install/setup.bash
   ```

<!--
## How to Use the Library
-->

## ROS集成测试

向 PX4 提交拉取请求（pull request）时，持续集成（CI）会运行该库的集成测试
这些测试用于验证模式注册、故障保护（failsafes）和模式替换功能是否按预期工作。

欲了解更多信息，请访问[PX4 ROS2 接口库集成测试](../test_and_ci/integration_testing_px4_ros2_interface.md)。
