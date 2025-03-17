# RoverAckermannSetpoint (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverAckermannSetpoint.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 forward_speed_setpoint 		  # [m/s] Desired speed in body x direction. Positiv: forwards, Negativ: backwards
float32 forward_speed_setpoint_normalized # [-1, 1] Desired normalized speed in body x direction. Positiv: forwards, Negativ: backwards
float32 steering_setpoint      		  # [rad] Desired steering angle
float32 steering_setpoint_normalized      # [-1, 1] Desired normalized steering angle
float32 lateral_acceleration_setpoint     # [m/s^2] Desired acceleration in body y direction. Positiv: right, Negativ: left.

# TOPICS rover_ackermann_setpoint

```
