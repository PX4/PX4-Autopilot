# Motor Failure System Plugin

This Gazebo plugin enables motor failure simulation for multirotor vehicles in PX4 SITL.

## Overview

The Motor Failure System plugin subscribes to a Gazebo Transport topic to receive motor failure commands and directly controls motor joints to simulate failures. This allows simulating motor failures during flight testing.

## Features

- Gazebo Transport integration for receiving motor failure commands
- Automatic detection of motor joints (rotor_0_joint, rotor_1_joint, etc.)
- Direct joint velocity override in PreUpdate cycle
- Thread-safe motor failure number handling
- Configurable topic names via SDF

## Configuration

### SDF Parameters

- `<MotorFailureTopic>` (optional): Gazebo Transport topic for receiving motor failure commands
  - Default: `/model/<model_name>/motor_failure/motor_number`
  - Follows Gazebo model-scoped topic naming convention
  - If specified: Uses the exact topic name provided

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

<!-- Custom topic name -->
<plugin
    filename="MotorFailurePlugin"
    name="gz::sim::systems::MotorFailureSystem">
    <MotorFailureTopic>/custom/topic/name</MotorFailureTopic>
</plugin>
```

## Usage

### Publishing Motor Failure Commands

To trigger a motor failure, publish a message to the Gazebo Transport topic.

For a vehicle with namespace `x500_0`:

```bash
# Fail motor 1 (motors are 1-indexed: 1, 2, 3, 4, ...)
gz topic -t /model/x500_0/motor_failure/motor_number -m gz.msgs.Int32 -p "data: 1"

# Clear motor failure (restore normal operation)
gz topic -t /model/x500_0/motor_failure/motor_number -m gz.msgs.Int32 -p "data: 0"
```

**Note**: Replace `x500_0` with your vehicle's model name.

**Motor Numbering**:
- Motors are **1-indexed**: 1, 2, 3, 4, etc.
- `data: 0` clears the motor failure
- `data: -1` also clears the motor failure

### Monitoring Motor Failure Status

You can monitor the motor failure messages in the Gazebo console output.


## Notes

- The plugin applies motor failure in the PreUpdate cycle by setting joint velocity to 0
- **Plugin Declaration Order**: This plugin must be declared AFTER the MulticopterMotorModel plugin in the SDF file to ensure proper execution order
