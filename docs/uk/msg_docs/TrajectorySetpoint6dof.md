---
pageClass: is-wide-page
---

# TrajectorySetpoint6dof (UORB message)

Trajectory setpoint in NED frame. Input to position controller.

**TOPICS:** trajectory_setpoint6dof

## Fields

| Назва                                 | Тип          | Unit [Frame] | Range/Enum | Опис                                                      |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                             | `uint64`     |                                                                  |            | time since system start (microseconds) |
| положення                             | `float32[3]` |                                                                  |            | in meters                                                 |
| швидкість                             | `float32[3]` |                                                                  |            | in meters/second                                          |
| acceleration                          | `float32[3]` |                                                                  |            | in meters/second^2                                        |
| jerk                                  | `float32[3]` |                                                                  |            | in meters/second^3 (for logging only)  |
| quaternion                            | `float32[4]` |                                                                  |            | unit quaternion                                           |
| angular_velocity | `float32[3]` |                                                                  |            | angular velocity in radians/second                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TrajectorySetpoint6dof.msg)

:::details
Click here to see original file

```c
# Trajectory setpoint in NED frame
# Input to position controller.

uint64 timestamp # time since system start (microseconds)

# NED local world frame
float32[3] position               # in meters
float32[3] velocity               # in meters/second
float32[3] acceleration           # in meters/second^2
float32[3] jerk                   # in meters/second^3 (for logging only)

float32[4] quaternion             # unit quaternion
float32[3] angular_velocity       # angular velocity in radians/second
```

:::
