---
pageClass: is-wide-page
---

# VehicleLocalPositionSetpoint (UORB message)

Local position setpoint in NED frame. Telemetry of PID position controller to monitor tracking. NaN means the state was not controlled.

**TOPICS:** vehicle_local_position_setpoint

## Fields

| Name                                      | Type         | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------------- | ------------ | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp       | `uint64`     |              |            | time since system start (microseconds) |
| <a id="fld_x"></a>x                       | `float32`    |              |            | in meters NED                          |
| <a id="fld_y"></a>y                       | `float32`    |              |            | in meters NED                          |
| <a id="fld_z"></a>z                       | `float32`    |              |            | in meters NED                          |
| <a id="fld_vx"></a>vx                     | `float32`    |              |            | in meters/sec                          |
| <a id="fld_vy"></a>vy                     | `float32`    |              |            | in meters/sec                          |
| <a id="fld_vz"></a>vz                     | `float32`    |              |            | in meters/sec                          |
| <a id="fld_acceleration"></a>acceleration | `float32[3]` |              |            | in meters/sec^2                        |
| <a id="fld_thrust"></a>thrust             | `float32[3]` |              |            | normalized thrust vector in NED        |
| <a id="fld_yaw"></a>yaw                   | `float32`    |              |            | in radians NED -PI..+PI                |
| <a id="fld_yawspeed"></a>yawspeed         | `float32`    |              |            | in radians/sec                         |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleLocalPositionSetpoint.msg)

::: details Click here to see original file

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
