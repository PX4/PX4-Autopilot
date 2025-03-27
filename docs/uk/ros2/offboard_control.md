# ROS 2 Offboard Control Приклад

The following C++ example shows how to do position control in [offboard mode](../flight_modes/offboard.md) from a ROS 2 node.

Приклад починає відправляти установочні точки, входить у режим offboard, озброюється, піднімається на 5 метрів і чекає.
Незважаючи на простоту, він демонструє основні принципи використання offboard control і способи надсилання команд транспортному засобу.

It has been tested on Ubuntu 20.04 with ROS 2 Foxy and PX4 v1.14.

:::warning
_Offboard_ control is dangerous.
Якщо ви керуєте реальним транспортним засобом, то обов'язково майте можливість отримати назад ручне керування на випадок, якщо щось піде не так.
:::

:::info
ROS and PX4 make a number of different assumptions, in particular with respect to [frame conventions](../ros/external_position_estimation.md#reference-frames-and-ros).
Не існує неявного перетворення між типами кадрів при публікації або підписці на теми!

У цьому прикладі публікуються позиції у фреймі NED, як і очікує PX4.
To subscribe to data coming from nodes that publish in a different frame (for example the ENU, which is the standard frame of reference in ROS/ROS 2), use the helper functions in the [frame_transforms](https://github.com/PX4/px4_ros_com/blob/main/src/lib/frame_transforms.cpp) library.
:::

## Випробування

Follow the instructions in [ROS 2 User Guide](../ros2/user_guide.md) to install PX and run the simulator, install ROS 2, and start the XRCE-DDS Agent.

After that we can follow a similar set of steps to those in [ROS 2 User Guide > Build ROS 2 Workspace](../ros2/user_guide.md#build-ros-2-workspace) to run the example.

:::tip
Make sure that QGC is connected to PX4 before running the ROS 2 node.
This is needed because, by default, you cannot arm a vehicle without a connection to ground station (QGC) or an established RC connection (this ensures there is always the option to regain manual control).
:::

Створити та запустити приклад:

1. Відкрийте новий термінал.

2. Створіть новий каталог робочого простору colcon і перейдіть до нього за допомогою:

 ```sh
 mkdir -p ~/ws_offboard_control/src/
 cd ~/ws_offboard_control/src/
 ```

3. Clone the [px4_msgs](https://github.com/PX4/px4_msgs) repo to the `/src` directory (this repo is needed in every ROS 2 PX4 workspace!):

 ```sh
 git clone https://github.com/PX4/px4_msgs.git
 # checkout the matching release branch if not using PX4 main.
 ```

4. Clone the example repository [px4_ros_com](https://github.com/PX4/px4_ros_com) to the `/src` directory:

 ```sh
 git clone https://github.com/PX4/px4_ros_com.git
 ```

5. Source the ROS 2 development environment into the current terminal and compile the workspace using `colcon`:

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

6. Source the `local_setup.bash`:

 ```sh
 source install/local_setup.bash
 ```

7. Запустіть приклад.

 ```
 ros2 run px4_ros_com offboard_control
 ```

Транспортний засіб повинен озброїтися, піднятися на 5 метрів і потім зачекати (вічно).

## Імплементація

The source code of the offboard control example can be found in [PX4/px4_ros_com](https://github.com/PX4/px4_ros_com) in the directory [/src/examples/offboard/offboard_control.cpp](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp).

:::info
PX4 publishes all the messages used in this example as ROS topics by default (see [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml)).
:::

PX4 requires that the vehicle is already receiving `OffboardControlMode` messages before it will arm in offboard mode, or before it will switch to offboard mode when flying.
In addition, PX4 will switch out of offboard mode if the stream rate of `OffboardControlMode` messages drops below approximately 2Hz.
Необхідна поведінка реалізується за допомогою головного циклу, що виконується у вузлі ROS 2, як показано нижче:

```cpp
auto timer_callback = [this]() -> void {

	if (offboard_setpoint_counter_ == 10) {
		// Change to Offboard mode after 10 setpoints
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

		// Arm the vehicle
		this->arm();
	}

	// OffboardControlMode needs to be paired with TrajectorySetpoint
	publish_offboard_control_mode();
	publish_trajectory_setpoint();

	// stop the counter after reaching 11
	if (offboard_setpoint_counter_ < 11) {
		offboard_setpoint_counter_++;
	}
};
timer_ = this->create_wall_timer(100ms, timer_callback);
```

Цикл працює на таймері тривалістю 100 мс.
For the first 10 cycles it calls `publish_offboard_control_mode()` and `publish_trajectory_setpoint()` to send [OffboardControlMode](../msg_docs/OffboardControlMode.md) and [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md) messages to PX4.
The `OffboardControlMode` messages are streamed so that PX4 will allow arming once it switches to offboard mode, while the `TrajectorySetpoint` messages are ignored (until the vehicle is in offboard mode).

After 10 cycles `publish_vehicle_command()` is called to change to offboard mode, and `arm()` is called to arm the vehicle.
Після того, як транспортний засіб озброюється та змінює режим, він починає відстежувати встановлені точки.
Задані точки все ще відправляються в кожному циклі, щоб транспортний засіб не виходив з режиму поза платформою.

The implementations of the `publish_offboard_control_mode()` and `publish_trajectory_setpoint()` methods are shown below.
These publish the [OffboardControlMode](../msg_docs/OffboardControlMode.md) and [TrajectorySetpoint](../msg_docs/TrajectorySetpoint.md) messages to PX4 (respectively).

The `OffboardControlMode` is required in order to inform PX4 of the _type_ of offboard control behing used.
Here we're only using _position control_, so the `position` field is set to `true` and all the other fields are set to `false`.

```cpp
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}
```

`TrajectorySetpoint` provides the position setpoint.
In this case, the `x`, `y`, `z` and `yaw` fields are hardcoded to certain values, but they can be updated dynamically according to an algorithm or even by a subscription callback for messages coming from another node.

```cpp
/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
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

The `publish_vehicle_command()` sends [VehicleCommand](../msg_docs/VehicleCommand.md) messages with commands to the flight controller.
We use it above to change the mode to offboard mode, and also in `arm()` to arm the vehicle.
While we don't call `disarm()` in this example, it is also used in the implementation of that function.

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
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}
```

:::info
[VehicleCommand](../msg_docs/VehicleCommand.md) is one of the simplest and most powerful ways to command PX4, and by subscribing to [VehicleCommandAck](../msg_docs/VehicleCommandAck.md) you can also confirm that setting a particular command was successful.
The param and command fields map to [MAVLink commands](https://mavlink.io/en/messages/common.html#mav_commands) and their parameter values.
:::

## Дивіться також

- [Python ROS2 offboard examples with PX4](https://github.com/Jaeyoung-Lim/px4-offboard) (Jaeyoung-Lim/px4-offboard).
