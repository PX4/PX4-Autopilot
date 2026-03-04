---
pageClass: is-wide-page
---

# RoverSteeringSetpoint (UORB message)

Rover Steering setpoint.

**TOPICS:** rover_steeringsetpoint

## Fields

| 参数名                                                                    | 类型        | Unit [Frame] | Range/Enum                                                                                                                        | 描述                                                                                                                                                        |
| ---------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                              | `uint64`  | us                                                               |                                                                                                                                   | Time since system start                                                                                                                                   |
| normalized_steering_setpoint | `float32` | [Body]       | [-1 (Left) : 1 (Right)] | Ackermann: Normalized steering angle, Differential/Mecanum: Normalized speed difference between the left and right wheels |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverSteeringSetpoint.msg)

:::details
Click here to see original file

```c
# Rover Steering setpoint

uint64 timestamp                      # [us] Time since system start
float32 normalized_steering_setpoint  # [-] [@range -1 (Left), 1 (Right)] [@frame Body] Ackermann: Normalized steering angle, Differential/Mecanum: Normalized speed difference between the left and right wheels
```

:::
