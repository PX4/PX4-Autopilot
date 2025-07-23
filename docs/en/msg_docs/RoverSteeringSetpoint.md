# RoverSteeringSetpoint (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverSteeringSetpoint.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 normalized_steering_angle # [-1, 1] Normalized steering angle (Ackermann only, Positiv: Steer right, Negativ: Steer left)

float32 normalized_speed_diff     # [-1, 1] Normalized speed difference between the left and right wheels of the rover (Differential/Mecanum only, Positiv = Turn right, Negativ: Turn left)

```
