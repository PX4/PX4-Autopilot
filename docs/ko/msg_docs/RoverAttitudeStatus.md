---
pageClass: is-wide-page
---

# RoverAttitudeStatus (UORB message)

Rover Attitude Status.

**TOPICS:** rover_attitudestatus

## Fields

| 명칭                                                              | 형식        | Unit [Frame] | Range/Enum                                                                     | 설명                                                                         |
| --------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------ | -------------------------------------------------------------------------- |
| timestamp                                                       | `uint64`  | us                                                               |                                                                                | Time since system start                                                    |
| measured_yaw                               | `float32` | rad [NED]    | [-pi : pi] | Measured yaw                                                               |
| adjusted_yaw_setpoint | `float32` | rad [NED]    | [-pi : pi] | Yaw setpoint that is being tracked (Applied slew rates) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverAttitudeStatus.msg)

:::details
Click here to see original file

```c
# Rover Attitude Status

uint64 timestamp               # [us] Time since system start
float32 measured_yaw           # [rad] [@range -pi, pi] [@frame NED]Measured yaw
float32 adjusted_yaw_setpoint  # [rad] [@range -pi, pi] [@frame NED] Yaw setpoint that is being tracked (Applied slew rates)
```

:::
