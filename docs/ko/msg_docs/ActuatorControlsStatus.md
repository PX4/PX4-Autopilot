---
pageClass: is-wide-page
---

# ActuatorControlsStatus (UORB message)

**TOPICS:** actuator_controls_status_0 actuator_controls_status_1

## Fields

| 명칭                                 | 형식           | Unit [Frame] | Range/Enum | 설명                                                        |
| ---------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                          | `uint64`     |                                                                  |            | time since system start (microseconds) |
| control_power | `float32[3]` |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorControlsStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp			# time since system start (microseconds)

float32[3] control_power

# TOPICS actuator_controls_status_0 actuator_controls_status_1
```

:::
