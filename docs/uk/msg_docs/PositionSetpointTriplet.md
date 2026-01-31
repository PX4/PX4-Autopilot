# PositionSetpointTriplet (Повідомлення UORB)

Глобальний набір точки встановлення у форматі координат WGS84.
Ось наступні три способи вказівань (або просто наступні два або один).

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PositionSetpointTriplet.msg)

```c
# Global position setpoint triplet in WGS84 coordinates.
# This are the three next waypoints (or just the next two or one).

uint64 timestamp		# time since system start (microseconds)

PositionSetpoint previous
PositionSetpoint current
PositionSetpoint next

```
