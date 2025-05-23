# RoverVelocityStatus (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverVelocityStatus.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 measured_speed_body_x          # [m/s] Measured speed in body x direction. Positiv: forwards, Negativ: backwards
float32 speed_body_x_setpoint          # [m/s] Speed setpoint in body x direction. Positiv: forwards, Negativ: backwards
float32 adjusted_speed_body_x_setpoint # [m/s] Post slew rate speed setpoint in body x direction. Positiv: forwards, Negativ: backwards
float32 measured_speed_body_y          # [m/s] Measured speed in body y direction. Positiv: right, Negativ: left
float32 speed_body_y_setpoint          # [m/s] Speed setpoint in body y direction. Positiv: right, Negativ: left (Only for mecanum rovers)
float32 adjusted_speed_body_y_setpoint # [m/s] Post slew rate speed setpoint in body y direction. Positiv: right, Negativ: left (Only for mecanum rovers)
float32 pid_throttle_body_x_integral   # Integral of the PID for the closed loop controller of the speed in body x direction
float32 pid_throttle_body_y_integral   # Integral of the PID for the closed loop controller of the speed in body y direction

# TOPICS rover_velocity_status

```
