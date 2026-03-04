---
pageClass: is-wide-page
---

# RoverRateSetpoint (UORB message)

Rover Rate setpoint.

**TOPICS:** rover_ratesetpoint

## Fields

| 명칭                                                          | 형식        | Unit [Frame] | Range/Enum                                                                       | 설명                      |
| ----------------------------------------------------------- | --------- | ---------------------------------------------------------------- | -------------------------------------------------------------------------------- | ----------------------- |
| timestamp                                                   | `uint64`  | us                                                               |                                                                                  | Time since system start |
| yaw_rate_setpoint | `float32` | rad/s [NED]  | [-inf : inf] | Yaw rate setpoint       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverRateSetpoint.msg)

:::details
Click here to see original file

```c
# Rover Rate setpoint

uint64 timestamp           # [us] Time since system start
float32 yaw_rate_setpoint  # [rad/s] [@range -inf, inf] [@frame NED] Yaw rate setpoint
```

:::
