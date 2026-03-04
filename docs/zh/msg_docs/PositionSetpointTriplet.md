---
pageClass: is-wide-page
---

# PositionSetpointTriplet (UORB message)

Global position setpoint triplet in WGS84 coordinates. This are the three next waypoints (or just the next two or one).

**TOPICS:** position_setpointtriplet

## Fields

| 参数名       | 类型                 | Unit [Frame] | Range/Enum | 描述                                                        |
| --------- | ------------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64`           |                                                                  |            | time since system start (microseconds) |
| previous  | `PositionSetpoint` |                                                                  |            |                                                           |
| current   | `PositionSetpoint` |                                                                  |            |                                                           |
| next      | `PositionSetpoint` |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PositionSetpointTriplet.msg)

:::details
Click here to see original file

```c
# Global position setpoint triplet in WGS84 coordinates.
# This are the three next waypoints (or just the next two or one).

uint64 timestamp		# time since system start (microseconds)

PositionSetpoint previous
PositionSetpoint current
PositionSetpoint next
```

:::
