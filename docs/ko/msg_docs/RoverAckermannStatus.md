# RoverAckermannStatus (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverAckermannStatus.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 measured_forward_speed       	      # [m/s] Measured speed in body x direction. Forwards: positiv, Backwards: negativ
float32 adjusted_forward_speed_setpoint       # [m/s] Speed setpoint after applying slew rate
float32 steering_setpoint_normalized          # [-1, 1] Normalized steering setpoint
float32 adjusted_steering_setpoint_normalized # [-1, 1] Normalized steering setpoint after applying slew rate
float32 measured_lateral_acceleration         # [m/s^2] Measured acceleration in body y direction. Positiv: right, Negativ: left.
float32 pid_throttle_integral                 # Integral of the PID for the closed loop speed controller
float32 pid_lat_accel_integral                # Integral of the PID for the closed loop lateral acceleration controller

# TOPICS rover_ackermann_status

```
