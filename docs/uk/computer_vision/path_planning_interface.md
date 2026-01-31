# Інтерфейс планування маршруту

<Badge type="warning" text="Removed PX4 v1.15" />

:::warning
The **Path Planning Interface**, along with the features **Obstacle avoidance in Missions** and **Safe Landing** are no longer supported or maintained, and _should not_ be used in any PX4 version.

This code was abandoned due to architectural constraints of the implementation making it hard to maintain, extend, and adopt.
Support has been withdrawn make it clear that this interface is untested.
:::

:::tip
PX4 is now adopting more generic and scalable approaches for integrating these kinds of features.
For example the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) allows complete replacement of PX4 flight modes with enhanced versions written using ROS 2.
:::

This interface allows PX4 to stream a proposed path to a companion computer, and receive back a stream of setpoints that more safely achieves the emitted path, or a mirror of the same stream if the path planning software does not support planning for the current PX4 mode.
This enables features such obstacle avoidance in missions and safer landing to be provided by a planner on a companion computer.

This actual code is still present in code at time of writing (PX4 v1.15).
Information about the API and associated features can be found in the [PX4 v1.14 docs](https://docs.px4.io/v1.14/en/computer_vision/path_planning_interface.html).
