---
pageClass: is-wide-page
---

# VelocityLimits (UORB message)

Velocity and yaw rate limits for a multicopter position slow mode only.

**TOPICS:** velocity_limits

## Fields

| 명칭                                       | 형식        | Unit [Frame] | Range/Enum | 설명                                                        |
| ---------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                | `uint64`  |                                                                  |            | time since system start (microseconds) |
| horizontal_velocity | `float32` | m/s                                                              |            |                                                           |
| vertical_velocity   | `float32` | m/s                                                              |            |                                                           |
| yaw_rate            | `float32` | rad/s                                                            |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VelocityLimits.msg)

:::details
Click here to see original file

```c
# Velocity and yaw rate limits for a multicopter position slow mode only

uint64 timestamp # time since system start (microseconds)

# absolute speeds, NAN means use default limit
float32 horizontal_velocity # [m/s]
float32 vertical_velocity # [m/s]
float32 yaw_rate # [rad/s]
```

:::
