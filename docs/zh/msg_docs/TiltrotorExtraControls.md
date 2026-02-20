---
pageClass: is-wide-page
---

# TiltrotorExtraControls (UORB message)

**TOPICS:** tiltrotor_extracontrols

## Fields

| 参数名                                                                                                  | 类型        | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                  |
| ---------------------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                            | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                                                           |
| collective_tilt_normalized_setpoint   | `float32` |                                                                  |            | Collective tilt angle of motors of tiltrotor, 0: vertical, 1: horizontal [0, 1] |
| collective_thrust_normalized_setpoint | `float32` |                                                                  |            | Collective thrust setpoint [0, 1]                                                                               |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TiltrotorExtraControls.msg)

:::details
Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)

float32 collective_tilt_normalized_setpoint	# Collective tilt angle of motors of tiltrotor, 0: vertical, 1: horizontal [0, 1]
float32 collective_thrust_normalized_setpoint 	# Collective thrust setpoint [0, 1]
```

:::
