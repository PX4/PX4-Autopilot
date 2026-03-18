---
pageClass: is-wide-page
---

# WheelEncoders (UORB message)

**TOPICS:** wheel_encoders

## Fields

| 명칭                               | 형식           | Unit [Frame] | Range/Enum | 설명                                                        |
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
