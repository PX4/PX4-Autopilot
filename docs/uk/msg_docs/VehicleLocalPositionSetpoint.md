# VehicleLocalPositionSetpoint (повідомлення UORB)

Місцева позиція задана в рамці NED
Телеметрія контролера позиції PID для відстеження.
NaN означає, що стан не був контрольований

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleLocalPositionSetpoint.msg)

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
