# Apps & API

The rover modules have been tested and integrated with a subset of the available [Apps & API](../middleware/index.md) methods.
We specifically provide guides for using [ROS 2](../ros2/index.md) to interface a companion computer with PX4 via [uXRCE-DDS](../middleware/uxrce_dds.md).

| Method                                                                       | Опис                                                                                                                                                                                                   |
| ---------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [PX4 ROS 2 Interface](#px4-ros-2-interface) (Recommended) | Register a custom mode and publish [RoverSetpointTypes](../ros2/px4_ros2_control_interface.md#rover-setpoints).                                                                        |
| [ROS 2 Offboard Control](#ros-2-offboard-control)                            | Use the PX4 internal [Offboard Mode](../flight_modes/offboard.md) and publish messages defined in [dds_topics.yaml](../middleware/dds_topics.md). |

## PX4 ROS 2 Interface

We recommend the use of the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) which allows you to register a custom drive mode and exposes [RoverSetpointTypes](../ros2/px4_ros2_control_interface.md#rover-setpoints).

By using these setpoints (instead of the PX4 internal rover setpoints), you are guaranteed to send valid control inputs to your vehicle and the control flags for the setpoints you are using are automatically set for you.
Registering a custom drive mode instead of using [ROS 2 Offboard Control](#ros-2-offboard-control) additionally provides the advantages listed [here](../concept/flight_modes.md#internal-vs-external-modes).

To get familiar with this method, read through the guide for the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) where we also provide an example app for rover.

## ROS 2 Offboard Control

[ROS 2 Offboard Control](../ros2/offboard_control.md) uses the PX4 internal [Offboard Mode](../flight_modes/offboard.md).

While you can subscribe/publish to all topics specified in [dds_topics.yaml](../middleware/dds_topics.md), not all rover modules support all of these topics (see [Supported Setpoints](../flight_modes/offboard.md#rover)).
Unlike the [RoverSetpointTypes](../ros2/px4_ros2_control_interface.md#rover-setpoints) exposed through the [PX4 ROS 2 Interface](#px4-ros-2-interface), there is no guarantee that the published setpoints lead to a valid control input.

In addition, the correct control mode flags must be set through [OffboardControlMode](../msg_docs/OffboardControlMode.md).
This requires a deeper understanding of PX4 and the structure of the rover modules.
For general information on setting up offboard mode read through [Offboard Mode](../flight_modes/offboard.md) and then consult [Supported Setpoints](../flight_modes/offboard.md#rover).
