---
pageClass: is-wide-page
---

# RoverRateStatus (UORB message)

Rover Rate Status.

**TOPICS:** rover_ratestatus

## Fields

| 명칭                                                                                        | 형식        | Unit [Frame] | Range/Enum                                                                       | 설명                                                                              |
| ----------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | -------------------------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| timestamp                                                                                 | `uint64`  | us                                                               |                                                                                  | Time since system start                                                         |
| measured_yaw_rate                               | `float32` | rad/s [NED]  | [-inf : inf] | Measured yaw rate                                                               |
| adjusted_yaw_rate_setpoint | `float32` | rad/s [NED]  | [-inf : inf] | Yaw rate setpoint that is being tracked (Applied slew rates) |
| pid_yaw_rate_integral      | `float32` |                                                                  | [-1 : 1]     | Integral of the PID for the closed loop yaw rate controller                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverRateStatus.msg)

:::details
Click here to see original file

```c
# Rover Rate Status

uint64 timestamp                    # [us] Time since system start
float32 measured_yaw_rate           # [rad/s] [@range -inf, inf] [@frame NED] Measured yaw rate
float32 adjusted_yaw_rate_setpoint  # [rad/s] [@range -inf, inf] [@frame NED] Yaw rate setpoint that is being tracked (Applied slew rates)
float32 pid_yaw_rate_integral       # [-] [@range -1, 1] Integral of the PID for the closed loop yaw rate controller
```

:::
