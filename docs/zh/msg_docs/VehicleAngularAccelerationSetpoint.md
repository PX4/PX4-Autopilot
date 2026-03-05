---
pageClass: is-wide-page
---

# VehicleAngularAccelerationSetpoint (UORB message)

**TOPICS:** vehicle_angularacceleration_setpoint

## Fields

| 参数名                                   | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                            |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`     |                                                                  |            | time since system start (microseconds)                                     |
| timestamp_sample | `uint64`     |                                                                  |            | timestamp of the data sample on which this message is based (microseconds) |
| xyz                                   | `float32[3]` |                                                                  |            | angular acceleration about X, Y, Z body axis in rad/s^2                                       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleAngularAccelerationSetpoint.msg)

:::details
Click here to see original file

```c
uint64 timestamp         # time since system start (microseconds)
uint64 timestamp_sample  # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz           # angular acceleration about X, Y, Z body axis in rad/s^2
```

:::
