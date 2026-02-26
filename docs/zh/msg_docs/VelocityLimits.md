---
pageClass: is-wide-page
---

# 速度限制 (UORB 消息)

Velocity and yaw rate limits for a multicopter position slow mode only.

**TOPICS:** velocity_limits

## Fields

| 参数名                                      | 类型        | Unit [Frame] | Range/Enum | 描述                                                        |
| ---------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                | `uint64`  |                                                                  |            | time since system start (microseconds) |
| horizontal_velocity | `float32` | 米/秒                                                              |            |                                                           |
| vertical_velocity   | `float32` | 米/秒                                                              |            |                                                           |
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
