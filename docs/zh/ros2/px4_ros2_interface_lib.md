# PX4 ROS 2 Interface Library

<Badge type="tip" text="PX4 v1.15" /> <Badge type="warning" text="Experimental" />

:::warning
Experimental
At the time of writing, parts of the PX4 ROS 2 Interface Library are experimental, and hence subject to change.
:::

The [PX4 ROS 2 Interface Library](https://github.com/Auterion/px4-ros2-interface-lib) is a C++ library that simplifies controlling and interacting with PX4 from ROS 2.

The library provides two high-level interfaces for developers:

1. The [Control Interface](./px4_ros2_control_interface.md) allows developers to create and dynamically register modes written using ROS 2.
   It provides classes for sending different types of setpoints, ranging from high-level navigation tasks all the way down to direct actuator controls.
2. The [Navigation Interface](./px4_ros2_navigation_interface.md) enables sending vehicle position estimates to PX4 from ROS 2 applications, such as a VIO system.

<!--
## Overview
-->

## Installation in a ROS 2 Workspace

To get started using the library within an existing ROS 2 workspace:

1. Make sure you have a working [ROS 2 setup](../ros2/user_guide.md), with [`px4_msgs`](https://github.com/PX4/px4_msgs) in the ROS 2 workspace.

2. Clone the repository into the workspace:

   ```sh
   cd $ros_workspace/src
   git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib
   ```

   ::: info
   To ensure compatibility, use the latest _main_ branches for PX4, _px4_msgs_ and the library.
   See also [here](https://github.com/Auterion/px4-ros2-interface-lib#compatibility-with-px4).

:::

3. Build the workspace:

   ```sh
   cd ..
   colcon build
   source install/setup.bash
   ```

<!--
## How to Use the Library
-->

## CI: Integration Tests

When opening a pull request to PX4, CI runs the library integration tests.
These test that mode registration, failsafes, and mode replacement, work as expected.

The tests can also be run locally from PX4:

```sh
./test/ros_test_runner.py
```

And to run only a single case:

```sh
./test/ros_test_runner.py --verbose --case <case>
```

You can list the available test cases with:

```sh
./test/ros_test_runner.py --list-cases
```
