# WheelEncoders (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/WheelEncoders.msg)

```c
uint64 timestamp			# time since system start (microseconds)

# Two wheels: 0 right, 1 left
float32[2] wheel_speed # [rad/s]
float32[2] wheel_angle # [rad]

```
