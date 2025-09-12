# Drive Modes

Flight modes (or more accurately "Drive modes" for ground vehicles) provide autopilot support to make it easier to manually drive the vehicle or to execute autonomous missions.

This section outlines all supported drive modes for [Rovers](../frames_rover/index.md).

For information on mapping RC control switches to specific modes see: [Basic Configuration > Flight Modes](../config/flight_mode.md).

::: warning
Selecting any other mode than those listed below will either stop the rover or can lead to undefined behaviour.
:::

## Manual Modes

| Mode                                    | Description                                                                                                                                                                      |
| --------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Manual](manual.md#manual-mode)         | No autopilot support. User is responsible for keeping the rover on the desired course and maintaining speed and rate of turn.                                                    |
| [Acro](manual.md#acro-mode)             | + Maintains the yaw rate (feels more like driving a car than manual mode). <br>+ Allows maximum yaw rate to be limited (protects against roll over).                             |
| [Stabilized](manual.md#stabilized-mode) | + Maintains the yaw (significantly better at holding a straight line).                                                                                                           |
| [Position](manual.md#position-mode)     | + Maintains the course (best mode for driving a straight line).<br>+ Maintains speed against disturbances, e.g. when driving up a hill.<br>+ Allows maximum speed to be limited. |

## Auto Modes

| Mode                            | Description                                                             |
| ------------------------------- | ----------------------------------------------------------------------- |
| [Mission](auto.md#mission-mode) | Automatic mode that causes the vehicle to execute a predefined mission. |
| [Return](auto.md#return-mode)   | Automatic mode that returns the vehicle to the launch position.         |

## Apps & API

The rover modules have been tested and integrated with a subset of the available [Apps & API](../middleware/index.md) methods.
We specifically provide guides for using [ROS 2](../ros2/index.md) to interface a companion computer with PX4 via [uXRCE-DDS](../middleware/uxrce_dds.md).

| Method                                                          | Description                                                                                                                                       |
| --------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| (Recommended) [PX4 ROS 2 Interface](api.md#px4-ros-2-interface) | Register a custom mode and publish [RoverSetpointTypes](../ros2/px4_ros2_control_interface.md#experimental-rover-setpoints).                      |
| [ROS 2 Offboard Control](api.md#ros-2-offboard-control)         | Use the PX4 internal [Offboard Mode](../flight_modes/offboard.md) and publish messages defined in [dds_topics.yaml](../middleware/dds_topics.md). |
