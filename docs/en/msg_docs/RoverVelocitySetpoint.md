# RoverVelocitySetpoint (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverVelocitySetpoint.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 speed   # [m/s] [-inf, inf] Speed setpoint (Backwards driving if negative)
float32 bearing # [rad] [-pi,pi] from North. [invalid: NAN, speed is defined in body x direction]
float32 yaw 	# [rad] [-pi, pi] (Mecanum only, Optional, defaults to current vehicle yaw) Vehicle yaw setpoint in NED frame

```
