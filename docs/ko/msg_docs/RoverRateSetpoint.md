# RoverRateSetpoint (UORB message)

Rover Rate setpoint

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverRateSetpoint.msg)

```c
# Rover Rate setpoint

uint64 timestamp           # [us] Time since system start
float32 yaw_rate_setpoint  # [rad/s] [@range -inf, inf] [@frame NED] Yaw rate setpoint

```
