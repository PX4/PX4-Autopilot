---
pageClass: is-wide-page
---

# RoverAttitudeSetpoint (UORB message)

Rover Attitude Setpoint.

**TOPICS:** rover_attitudesetpoint

## Fields

| 명칭                                | 형식        | Unit [Frame] | Range/Enum                                                                       | 설명                      |
| --------------------------------- | --------- | ---------------------------------------------------------------- | -------------------------------------------------------------------------------- | ----------------------- |
| timestamp                         | `uint64`  | us                                                               |                                                                                  | Time since system start |
| yaw_setpoint | `float32` | rad [NED]    | [-inf : inf] | Yaw setpoint            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverAttitudeSetpoint.msg)

:::details
Click here to see original file

```c
# Rover Attitude Setpoint

uint64 timestamp      # [us] Time since system start
float32 yaw_setpoint  # [rad] [@range -inf, inf] [@frame NED] Yaw setpoint
```

:::
