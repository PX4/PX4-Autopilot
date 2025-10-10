# Motor Failure System Plugin

This Gazebo plugin enables motor failure simulation for multirotor vehicles in PX4 SITL.

## Overview

The Motor Failure System plugin subscribes to a ROS2 topic to receive motor failure commands and directly controls motor joints to simulate failures. This allows simulating motor failures during flight testing.

## Features

- ROS2 integration for receiving motor failure commands
- Automatic detection of motor joints (rotor_0_joint, rotor_1_joint, etc.)
- Direct joint velocity override in PreUpdate cycle
- Thread-safe motor failure number handling
- Configurable topic names via SDF

## Configuration

### SDF Parameters

- `<robotNamespace>` (optional): Robot namespace
  - Default: entity/model name
  - This namespace is automatically prepended to the ROS2 topic name
- `<ROSMotorNumSubTopic>` (optional): ROS2 topic for receiving motor failure commands
  - Default: `/<robotNamespace>/motor_failure/motor_number`
  - If specified: Uses the exact topic name provided (no namespace prepended)

### Example SDF Usage

**IMPORTANT**: The MotorFailureSystem plugin must be declared **AFTER** the `gz-sim-multicopter-motor-model-system` plugin in your SDF file. This ensures the motor failure override runs after the motor model sets its velocity commands.

```xml
<!-- First: Motor model plugin -->
<plugin
    filename="gz-sim-multicopter-motor-model-system"
    name="gz::sim::systems::MulticopterMotorModel">
    <!-- motor model configuration -->
</plugin>

<!-- Second: Motor failure plugin (must come after motor model) -->

<!-- Minimal configuration (uses model name as namespace) -->
<plugin
    filename="MotorFailurePlugin"
    name="gz::sim::systems::MotorFailureSystem">
</plugin>

<!-- Custom configuration -->
<plugin
    filename="MotorFailurePlugin"
    name="gz::sim::systems::MotorFailureSystem">
    <robotNamespace>x500</robotNamespace>
    <ROSMotorNumSubTopic>/custom/topic/name</ROSMotorNumSubTopic>
</plugin>
```

## Usage

### Publishing Motor Failure Commands

To trigger a motor failure, publish a message to the ROS2 topic.

For a vehicle with namespace `x500_0`:

```bash
# Fail motor 1 (motors are 1-indexed: 1, 2, 3, 4, ...)
ros2 topic pub --once /x500_0/motor_failure/motor_number std_msgs/msg/Int32 "data: 1"

# Clear motor failure (restore normal operation)
ros2 topic pub --once /x500_0/motor_failure/motor_number std_msgs/msg/Int32 "data: 0"
```

**Note**: Replace `x500_0` with your vehicle's namespace (robotNamespace parameter).

**Motor Numbering**:
- Motors are **1-indexed**: 1, 2, 3, 4, etc.
- `data: 0` clears the motor failure
- `data: -1` also clears the motor failure

### Monitoring Motor Failure Status

You can monitor the motor failure messages in the Gazebo console output.

## Dependencies

- Gazebo Garden or later
- ROS2 (Humble or later)
- rclcpp
- std_msgs

## Building

This plugin requires ROS2 and is built conditionally using the `BUILD_ROS2_PLUGINS` CMake option.

### Building with Motor Failure Plugin (ROS2 Required)

1. Source your ROS2 installation:
```bash
source /opt/ros/humble/setup.bash  # or jazzy/rolling
```

2. Build PX4 with ROS2 plugins enabled:
```bash
cd PX4-Autopilot
CMAKE_ARGS="-DBUILD_ROS2_PLUGINS=ON" make px4_sitl gz_x500
```

### Building without Motor Failure Plugin

If you don't have ROS2 or don't need the motor failure plugin:

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

The motor failure plugin will not be built by default (BUILD_ROS2_PLUGINS=OFF).

## Notes

- The plugin automatically initializes ROS2 if not already initialized
- Motor failure number is thread-safe protected with a mutex
- The plugin applies motor failure in the PreUpdate cycle by setting joint velocity to 0
- **Plugin Declaration Order**: This plugin must be declared AFTER the MulticopterMotorModel plugin in the SDF file to ensure proper execution order
