---
pageClass: is-wide-page
---

# ActuatorArmed (UORB message)

**TOPICS:** actuator_armed

## Fields

| 参数名                                                                                    | 类型       | Unit [Frame] | Range/Enum | 描述                                                                                                 |
| -------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------- |
| timestamp                                                                              | `uint64` |                                                                  |            | time since system start (microseconds)                                          |
| armed                                                                                  | `bool`   |                                                                  |            | Set to true if system is armed                                                                     |
| prearmed                                                                               | `bool`   |                                                                  |            | Set to true if the actuator safety is disabled but motors are not armed                            |
| ready_to_arm                                 | `bool`   |                                                                  |            | Set to true if system is ready to be armed                                                         |
| lockdown                                                                               | `bool`   |                                                                  |            | Set to true if actuators are forced to being disabled (due to emergency or HIL) |
| kill                                                                                   | `bool`   |                                                                  |            | Set to true if manual throttle kill switch is engaged                                              |
| termination                                                                            | `bool`   |                                                                  |            | Send out failsafe (by default same as disarmed) output                          |
| in_esc_calibration_mode | `bool`   |                                                                  |            | IO/FMU should ignore messages from the actuator controls topics                                    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorArmed.msg)

:::details
Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)

bool armed		# Set to true if system is armed
bool prearmed		# Set to true if the actuator safety is disabled but motors are not armed
bool ready_to_arm	# Set to true if system is ready to be armed
bool lockdown		# Set to true if actuators are forced to being disabled (due to emergency or HIL)
bool kill               # Set to true if manual throttle kill switch is engaged
bool termination        # Send out failsafe (by default same as disarmed) output
bool in_esc_calibration_mode # IO/FMU should ignore messages from the actuator controls topics
```

:::
