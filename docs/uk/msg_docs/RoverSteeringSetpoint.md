# RoverSteeringSetpoint (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverSteeringSetpoint.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 normalized_steering_angle # [-1, 1] Normalized steering angle (Only for Ackermann-steered rovers)
float32 normalized_speed_diff     # [-1, 1] Normalized speed difference between the left and right wheels of the rover (Only for Differential/Mecanum rovers)

# TOPICS rover_steering_setpoint

```
