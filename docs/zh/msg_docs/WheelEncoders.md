---
pageClass: is-wide-page
---

# 滚轮编码器 (UORB 消息)

**TOPICS:** wheel_encoders

## Fields

| 参数名                              | 类型           | Unit [Frame] | Range/Enum | 描述                                                        |
| -------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                        | `uint64`     |                                                                  |            | time since system start (microseconds) |
| wheel_speed | `float32[2]` | rad/s                                                            |            |                                                           |
| wheel_angle | `float32[2]` | rad                                                              |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/WheelEncoders.msg)

:::details
Click here to see original file

```c
uint64 timestamp			# time since system start (microseconds)

# Two wheels: 0 right, 1 left
float32[2] wheel_speed # [rad/s]
float32[2] wheel_angle # [rad]
```

:::
