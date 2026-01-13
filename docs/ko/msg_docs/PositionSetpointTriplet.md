# PositionSetpointTriplet (UORB message)

Global position setpoint triplet in WGS84 coordinates.
This are the three next waypoints (or just the next two or one).

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PositionSetpointTriplet.msg)

```c
# Global position setpoint triplet in WGS84 coordinates.
# This are the three next waypoints (or just the next two or one).

uint64 timestamp		# time since system start (microseconds)

PositionSetpoint previous
PositionSetpoint current
PositionSetpoint next

```
