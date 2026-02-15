---
pageClass: is-wide-page
---

# VehicleAngularVelocity (UORB message)

**TOPICS:** vehicle_angular_velocity vehicle_angular_velocity_groundtruth

## Fields

| 명칭                                    | 형식           | Unit [Frame] | Range/Enum | 설명                                                                                            |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`     |                                                                  |            | time since system start (microseconds)                                     |
| timestamp_sample | `uint64`     |                                                                  |            | timestamp of the data sample on which this message is based (microseconds) |
| xyz                                   | `float32[3]` |                                                                  |            | Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s                    |
| xyz_derivative   | `float32[3]` |                                                                  |            | angular acceleration about the FRD body frame XYZ-axis in rad/s^2                             |

## Constants

| 명칭                                                                   | 형식       | Value | 설명 |
| -------------------------------------------------------------------- | -------- | ----- | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleAngularVelocity.msg)

:::details
Click here to see original file

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz		  # Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s

float32[3] xyz_derivative # angular acceleration about the FRD body frame XYZ-axis in rad/s^2

# TOPICS vehicle_angular_velocity vehicle_angular_velocity_groundtruth
```

:::
