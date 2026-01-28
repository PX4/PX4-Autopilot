# SensorPreflightMag (UORB message)

Pre-flight sensor check metrics.
The topic will not be updated when the vehicle is armed

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorPreflightMag.msg)

```c
#
# Pre-flight sensor check metrics.
# The topic will not be updated when the vehicle is armed
#
uint64 timestamp # time since system start (microseconds)

float32 mag_inconsistency_angle # maximum angle between magnetometer instance field vectors in radians.

```
