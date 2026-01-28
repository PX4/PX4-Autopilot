# RoverThrottleSetpoint (UORB message)

Rover Throttle setpoint

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverThrottleSetpoint.msg)

```c
# Rover Throttle setpoint

uint64 timestamp         # [us] Time since system start
float32 throttle_body_x  # [-] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Throttle setpoint along body X axis
float32 throttle_body_y  # [-] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Throttle setpoint along body Y axis

```
