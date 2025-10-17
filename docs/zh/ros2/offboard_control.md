# ROS 2 Offboard 控制示例

以下的 C++ 示例展示了如何在[offboard mode](../flight_modes/offboard.md)  中从 ROS 2 节点进行多轴位置控制。

示例将首先发送设置点、进入offboard模式、解锁、起飞至5米，并悬停等待。
虽然简单，但它显示了如何使用offboard控制以及如何向无人机发送指令。

该内容已在搭载 ROS 2 Foxy 与 PX4 v1.14 的 Ubuntu 20.04 系统上完成测试。

:::warning
_Offboard_ control is dangerous.
如果你是在一个真正的无人机平台上进行试验，请保证你已经设置了切换回手动的开关来防止紧急情况的发生。
:::

:::info
ROS 与 PX4 存在若干不同的预设（假设），尤其是在 [frame conventions](../ros/external_position_estimation.md#reference-frames-and-ros)
当主题发布或订阅时，坐标系类型之间没有隐含转换！

这个例子按照 PX4 的预期在NED坐标系下发布位置。
若要订阅来自在不同框架内发布的节点的数据(例如ENU, 这是ROS/ROS 2中的标准参考框架），使用[frame_transforms](https://github.com/PX4/px4_ros_com/blob/main/src/lib/frame_transforms.cpp)库中的辅助函数。
:::

## 小试身手

按照 [ROS 2 用户指南](../ros2/user_guide.md)中的说明来安装PX 并运行多轴模拟器，安装ROS 2, 并启动XRCE-DDS代理。

之后，我们可参照 [ROS 2 用户指南 > 构建 ROS 2 工作空间](../ros2/user_guide.md#build-ros-2-workspace)中的相似的步骤来运行这个例子。

:::tip
运行 ROS 2 节点前，请确保 QGC已连接到 PX4。
之所以需要这样做，是因为默认情况下，若未连接地面控制站（QGC）或已建立的RC连接，飞行器无法解锁（这一机制可确保始终存在重新获得手动控制权的途径）。
:::

构建并运行示例：

1. 打开一个新的终端。

2. 使用以下方法创建并切换至新的 colcon工作目录：

   ```sh
   mkdir -p ~/ws_offboard_control/src/
   cd ~/ws_offboard_control/src/
   ```

3. 将[px4_msgs](https://github.com/PX4/px4_msgs)代码仓库克隆到 /src 目录下（每个 ROS 2 PX4 工作空间都需要该仓库！）：

   ```sh
   git clone https://github.com/PX4/px4_msgs.git
   # checkout the matching release branch if not using PX4 main.
   ```

4. 将示例代码仓库 [px4_ros_com](https://github.com/PX4/px4_ros_com)克隆到 /src 目录下：

   ```sh
   git clone https://github.com/PX4/px4_ros_com.git
   ```

5. 在当前终端中加载 ROS 2 开发环境，并使用 colcon 工具编译工作空间：

   :::: tabs

   ::: tab humble

   ```sh
   cd ..
   source /opt/ros/humble/setup.bash
   colcon build
   ```


:::

   ::: tab foxy

   ```sh
   cd ..
   source /opt/ros/foxy/setup.bash
   colcon build
   ```


:::

   ::::

6. 来源 `local_setup.bash`：

   ```sh
   source install/local_setup.bash
   ```

7. 启动例程。

   ```
   ros2 run px4_ros_com offboard_control
   ```

飞行器将解锁、起飞至 5 米并悬停等待(永久)。

## 实现

离板控制示例的源代码可以在[ PX4/px4_ros_com ]目录里 [/src/examples/offboard/offboard_control.cpp](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp)中找到 [X4/px4_ros_com](https://github.com/PX4/px4_ros_com)。

:::info
PX4 默认情况下将此示例中使用的所有消息以ROS为话题发布(详见 [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml))。
:::

PX4 要求，飞行器需先持续接收 OffboardControlMode（离板控制模式）消息，之后才能在离板模式下解锁（arm），或在飞行过程中切换至离板模式。
此外，若 OffboardControlMode（离板控制模式）消息的数据流速率降至约 2Hz 以下，PX4 将会退出离板模式。
该行为在ROS 2 节点的主循环中实现的，如下所示：

```cpp
auto timer_callback = [this]() -&gt; void {

    if (offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        this-&gt;publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        this-&gt;arm();
    }

    // OffboardControlMode needs to be paired with TrajectorySetpoint
    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    // stop the counter after reaching 11
    if (offboard_setpoint_counter_ &lt; 11) {
        offboard_setpoint_counter_++;
    }
};
timer_ = this-&gt;create_wall_timer(100ms, timer_callback);
```

循环运行在一个100毫秒计时器。
在最初的 10 个循环中，它会调用 `publish_offboard_control_mode()` 和 `publish_trajectory_setpoint()` 这两个函数，向 PX4 发送 OffboardControlMode[OffboardControlMode](../msg_docs/OffboardControlMode.md) 和 [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md) 消息。
OffboardControlMode消息会持续发送，以便 PX4 切换到离板模式后允许解锁；而 TrajectorySetpoint消息会被忽略（直到载具处于离板模式）

10 个循环后，会调用 publish_vehicle_command() 函数切换至离板模式，并调用 arm() 函数对载具进行解锁。
在载具解锁并和切换模式后，它将开始跟踪位置设定值。
在每个周期内仍然发送设定值，确保载具不会切换出offboard模式。

publish_offboard_control_mode() 和 publish_trajectory_setpoint() 这两个方法的实现代码如下所示。
这些方法会分别发布到 PX4 的 [OffboardControlMode](../msg_docs/OffboardControlMode.md和 [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md) 消息。

OffboardControlMode（离板控制模式）消息是必需的，其作用是告知 PX4 当前所使用的离板控制类型。
此处我们仅使用位置控制，因此将 `position` 字段设为`true`，而所有其他字段均设为 `false`。

```cpp
/**
 * @short 发布离板控制模式。
 *在本示例中，仅位置控制和高度控制处于激活状态
 */
无效的离板控制：:publish_offboard_control_mode()
Power
	OffboardControlModel msg{}；
	msg.position = true；
	msg.veocity = false；
	msg. cceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.subust_and_torque = false;
	msg. irect_actuator = false;
	msg.timestamp = this->get_clock()->now ().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}
```

`TrattorySettpoint` 提供了位置设定点。
在这种情况下，`x`、`y`、`z`和`yaw`字段的值是硬编码为特定数值的。 但它们可以根据算法动态更新，甚至可以通过订阅回调函数来从另一个节点进行更新。

```cpp
/**
 *@brief 发布轨迹设定点

 在本示例中，该函数会发送一个轨迹设定点，使载具在 5 米高度悬停，并保持 180 度的偏航角。
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}
```

`publish_vehicle_command()` 将带有命令的 [VehicleCommand](../msg_docs/VehicleCommand.md)消息发送给载具。
我们使用上面的方法将模式切换为 offboard 模式，同时也在 arm() 函数中用它来对载具进行解锁。
我们在此示例中不调用 `disarm()` ，但它也用于执行此功能。

```cpp
/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this-&gt;get_clock()-&gt;now().nanoseconds() / 1000;
    vehicle_command_publisher_-&gt;publish(msg);
}
```

:::info
[VehicleCommand](../msg_docs/VehicleCommand.md) 是命令PX4的最简单和最高效的方式之一。 通过订阅 [VehicleCommandAck](../msg_docs/VehicleCommandAck.md)，您也可以确认设置特定命令是否成功。
参数字段和 指令字段对应于 [MAVLink commands](https://mavlink.io/en/messages/common.html#mav_commands)以及他们的参数值
:::

## See Also

- [Python ROS2 offboard examples with PX4](https://github.com/Jaeyoung-Lim/px4-offboard) (Jaeyoung-Lim/px4-offboard).
