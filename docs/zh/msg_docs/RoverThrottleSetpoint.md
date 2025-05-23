# RoverThrottleSetpoint (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverThrottleSetpoint.msg)

```c

uint64 timestamp        # time since system start (microseconds)

float32 throttle_body_x # throttle setpoint along body X axis [-1, 1]
float32 throttle_body_y # throttle setpoint along body Y axis [-1, 1]

# TOPICS rover_throttle_setpoint

```
