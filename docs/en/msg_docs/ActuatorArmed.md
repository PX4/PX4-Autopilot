---
pageClass: is-wide-page
---

# ActuatorArmed (UORB message)

**TOPICS:** actuator_armed

## Fields

| Name                                                            | Type     | Unit [Frame] | Range/Enum | Description                                                                     |
| --------------------------------------------------------------- | -------- | ------------ | ---------- | ------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                             | `uint64` |              |            | time since system start (microseconds)                                          |
| <a id="fld_armed"></a>armed                                     | `bool`   |              |            | Set to true if system is armed                                                  |
| <a id="fld_prearmed"></a>prearmed                               | `bool`   |              |            | Set to true if the actuator safety is disabled but motors are not armed         |
| <a id="fld_ready_to_arm"></a>ready_to_arm                       | `bool`   |              |            | Set to true if system is ready to be armed                                      |
| <a id="fld_lockdown"></a>lockdown                               | `bool`   |              |            | Set to true if actuators are forced to being disabled (due to emergency or HIL) |
| <a id="fld_kill"></a>kill                                       | `bool`   |              |            | Set to true if manual throttle kill switch is engaged                           |
| <a id="fld_termination"></a>termination                         | `bool`   |              |            | Send out failsafe (by default same as disarmed) output                          |
| <a id="fld_in_esc_calibration_mode"></a>in_esc_calibration_mode | `bool`   |              |            | IO/FMU should ignore messages from the actuator controls topics                 |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorArmed.msg)

::: details Click here to see original file

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
