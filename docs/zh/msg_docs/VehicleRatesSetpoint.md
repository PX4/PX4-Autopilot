---
pageClass: is-wide-page
---

# VehicleRatesSetpoint (UORB message)

**TOPICS:** vehicle_ratessetpoint

## Fields

| 参数名                                 | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                                     |
| ----------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------ |
| timestamp                           | `uint64`     |                                                                  |            | time since system start (microseconds)                                              |
| roll                                | `float32`    | rad/s                                                            |            | roll rate setpoint                                                                                     |
| pitch                               | `float32`    | rad/s                                                            |            | pitch rate setpoint                                                                                    |
| yaw                                 | `float32`    | rad/s                                                            |            | yaw rate setpoint                                                                                      |
| thrust_body    | `float32[3]` |                                                                  |            | Normalized thrust command in body NED frame [-1,1] |
| reset_integral | `bool`       |                                                                  |            | Reset roll/pitch/yaw integrals (navigation logic change)                            |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述 |
| -------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleRatesSetpoint.msg)

:::details
Click here to see original file

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp	# time since system start (microseconds)

# body angular rates in FRD frame
float32 roll		# [rad/s] roll rate setpoint
float32 pitch		# [rad/s] pitch rate setpoint
float32 yaw		# [rad/s] yaw rate setpoint

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] thrust_body	# Normalized thrust command in body NED frame [-1,1]

bool reset_integral # Reset roll/pitch/yaw integrals (navigation logic change)
```

:::
