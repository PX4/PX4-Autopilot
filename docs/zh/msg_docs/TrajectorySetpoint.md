---
pageClass: is-wide-page
---

# TrajectorySetpoint (UORB message)

Trajectory setpoint in NED frame. Input to PID position controller. Needs to be kinematically consistent and feasible for smooth flight. setting a value to NaN means the state should not be controlled.

**TOPICS:** trajectory_setpoint

## Fields

| 参数名          | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                  |
| ------------ | ------------ | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------- |
| timestamp    | `uint64`     |                                                                  |            | time since system start (microseconds)                           |
| 位置           | `float32[3]` |                                                                  |            | in meters                                                                           |
| 速度           | `float32[3]` |                                                                  |            | in meters/second                                                                    |
| acceleration | `float32[3]` |                                                                  |            | in meters/second^2                                                                  |
| jerk         | `float32[3]` |                                                                  |            | in meters/second^3 (for logging only)                            |
| yaw          | `float32`    |                                                                  |            | euler angle of desired attitude in radians -PI..+PI |
| yawspeed     | `float32`    |                                                                  |            | angular velocity around NED frame z-axis in radians/second                          |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述 |
| -------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/TrajectorySetpoint.msg)

:::details
Click here to see original file

```c
# Trajectory setpoint in NED frame
# Input to PID position controller.
# Needs to be kinematically consistent and feasible for smooth flight.
# setting a value to NaN means the state should not be controlled

uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

# NED local world frame
float32[3] position # in meters
float32[3] velocity # in meters/second
float32[3] acceleration # in meters/second^2
float32[3] jerk # in meters/second^3 (for logging only)

float32 yaw # euler angle of desired attitude in radians -PI..+PI
float32 yawspeed # angular velocity around NED frame z-axis in radians/second
```

:::
