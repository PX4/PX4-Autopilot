---
pageClass: is-wide-page
---

# PositionSetpointTriplet (Повідомлення UORB)

Глобальний набір точки встановлення у форматі координат WGS84. Ось наступні три способи вказівань (або просто наступні два або один).

**TOPICS:** position_setpointtriplet

## Fields

| Назва     | Тип                | Unit [Frame] | Range/Enum | Опис                                                      |
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
