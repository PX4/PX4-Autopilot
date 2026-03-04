---
pageClass: is-wide-page
---

# RaptorInput (UORB message)

Raptor Input.

**TOPICS:** raptor_input

## Fields

| 参数名                                   | 类型           | Unit [Frame] | Range/Enum                                                                   | 描述                                                                                                                                               |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| timestamp                             | `uint64`     | us                                                               |                                                                              | Time since system start                                                                                                                          |
| timestamp_sample | `uint64`     | us                                                               |                                                                              | Sampling timestamp of the data this control response is based on                                                                                 |
| active                                | `bool`       |                                                                  |                                                                              | Signals if the policy is active (aka publishing actuator_motors)                                         |
| 位置                                    | `float32[3]` | m [FLU]      |                                                                              | Position of the vehicle_local_position frame                                                           |
| orientation                           | `float32[4]` |                                                                  |                                                                              | Orientation in the vehicle_attitude frame but using the FLU convention as a unit quaternion (w, x, y, z) |
| linear_velocity  | `float32[3]` | m/s [FLU]    |                                                                              | Linear velocity in the vehicle_local_position frame                                                    |
| angular_velocity | `float32[3]` | rad/s [FLU]  |                                                                              | Angular velocity in the body frame                                                                                                               |
| previous_action  | `float32[4]` |                                                                  | [-1 : 1] | Previous action. Motor commands normalized to [-1, 1]                        |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述                                                               |
| -------------------------------------------------------------------- | -------- | - | ---------------------------------------------------------------- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |                                                                  |
| <a href="#ACTION_DIM"></a> ACTION_DIM           | `uint8`  | 4 | Policy output dimensionality (for quadrotors) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/RaptorInput.msg)

:::details
Click here to see original file

```c
# Raptor Input

# The exact inputs to the Raptor foundation policy.
# Having access to the exact inputs helps with debugging and post-hoc analysis.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Sampling timestamp of the data this control response is based on
bool active # Signals if the policy is active (aka publishing actuator_motors)
float32[3] position # [m] [@frame FLU] Position of the vehicle_local_position frame
float32[4] orientation # [-] Orientation in the vehicle_attitude frame but using the FLU convention as a unit quaternion (w, x, y, z)
float32[3] linear_velocity # [m/s] [@frame FLU] Linear velocity in the vehicle_local_position frame
float32[3] angular_velocity # [rad/s]  [@frame FLU] Angular velocity in the body frame
uint8 ACTION_DIM = 4 # Policy output dimensionality (for quadrotors)
float32[4] previous_action # [@range -1, 1] Previous action. Motor commands normalized to [-1, 1]

# TOPICS raptor_input
```

:::
