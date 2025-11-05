# Rpm (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Rpm.msg)

```c
uint64 timestamp # time since system start (microseconds)

# rpm values of 0.0 mean within a timeout there is no movement measured
float32 rpm_estimate # filtered revolutions per minute
float32 rpm_raw

```
