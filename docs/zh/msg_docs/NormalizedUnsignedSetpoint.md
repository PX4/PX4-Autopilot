---
pageClass: is-wide-page
---

# NormalizedUnsignedSetpoint (UORB message)

**TOPICS:** flaps_setpoint spoilers_setpoint

## Fields

| 参数名                                      | 类型        | Unit [Frame] | Range/Enum | 描述                                                        |
| ---------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                | `uint64`  |                                                                  |            | time since system start (microseconds) |
| normalized_setpoint | `float32` | 0, 1                                                             |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/NormalizedUnsignedSetpoint.msg)

:::details
Click here to see original file

```c
uint64 timestamp        		# time since system start (microseconds)

float32 normalized_setpoint          	# [0, 1]

# TOPICS flaps_setpoint spoilers_setpoint
```

:::
