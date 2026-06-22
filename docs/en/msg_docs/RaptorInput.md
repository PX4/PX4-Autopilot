---
pageClass: is-wide-page
---

# RaptorInput (UORB message)

Raptor Input.

**TOPICS:** raptor_input

## Fields

| Name                                              | Type         | Unit [Frame] | Range/Enum | Description                                                                                              |
| ------------------------------------------------- | ------------ | ------------ | ---------- | -------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`     | us           |            | Time since system start                                                                                  |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`     | us           |            | Sampling timestamp of the data this control response is based on                                         |
| <a id="fld_active"></a>active                     | `bool`       |              |            | Signals if the policy is active (aka publishing actuator_motors)                                         |
| <a id="fld_position"></a>position                 | `float32[3]` | m [FLU]      |            | Position of the vehicle_local_position frame                                                             |
| <a id="fld_orientation"></a>orientation           | `float32[4]` |              |            | Orientation in the vehicle_attitude frame but using the FLU convention as a unit quaternion (w, x, y, z) |
| <a id="fld_linear_velocity"></a>linear_velocity   | `float32[3]` | m/s [FLU]    |            | Linear velocity in the vehicle_local_position frame                                                      |
| <a id="fld_angular_velocity"></a>angular_velocity | `float32[3]` | rad/s [FLU]  |            | Angular velocity in the body frame                                                                       |
| <a id="fld_previous_action"></a>previous_action   | `float32[4]` |              | [-1 : 1]   | Previous action. Motor commands normalized to [-1, 1]                                                    |

## Constants

| Name                                          | Type     | Value | Description                                   |
| --------------------------------------------- | -------- | ----- | --------------------------------------------- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0     |
| <a id="#ACTION_DIM"></a> ACTION_DIM           | `uint8`  | 4     | Policy output dimensionality (for quadrotors) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/RaptorInput.msg)

::: details Click here to see original file

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
