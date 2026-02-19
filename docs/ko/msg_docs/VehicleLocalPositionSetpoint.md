---
pageClass: is-wide-page
---

# VehicleLocalPositionSetpoint (UORB message)

Local position setpoint in NED frame. Telemetry of PID position controller to monitor tracking. NaN means the state was not controlled.

**TOPICS:** vehicle_localposition_setpoint

## Fields

| 명칭           | 형식           | Unit [Frame] | Range/Enum | 설명                                                        |
| ------------ | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp    | `uint64`     |                                                                  |            | time since system start (microseconds) |
| x            | `float32`    |                                                                  |            | in meters NED                                             |
| y            | `float32`    |                                                                  |            | in meters NED                                             |
| z            | `float32`    |                                                                  |            | in meters NED                                             |
| vx           | `float32`    |                                                                  |            | in meters/sec                                             |
| vy           | `float32`    |                                                                  |            | in meters/sec                                             |
| vz           | `float32`    |                                                                  |            | in meters/sec                                             |
| acceleration | `float32[3]` |                                                                  |            | in meters/sec^2                                           |
| thrust       | `float32[3]` |                                                                  |            | normalized thrust vector in NED                           |
| yaw          | `float32`    |                                                                  |            | in radians NED -PI..+PI   |
| yawspeed     | `float32`    |                                                                  |            | in radians/sec                                            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleLocalPositionSetpoint.msg)

:::details
Click here to see original file

```c
# Local position setpoint in NED frame
# Telemetry of PID position controller to monitor tracking.
# NaN means the state was not controlled

uint64 timestamp	# time since system start (microseconds)

float32 x		# in meters NED
float32 y		# in meters NED
float32 z		# in meters NED

float32 vx		# in meters/sec
float32 vy		# in meters/sec
float32 vz		# in meters/sec

float32[3] acceleration # in meters/sec^2
float32[3] thrust	# normalized thrust vector in NED

float32 yaw		# in radians NED -PI..+PI
float32 yawspeed	# in radians/sec
```

:::
