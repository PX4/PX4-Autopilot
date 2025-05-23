uint64 timestamp # time since system start (microseconds)

float32 measured_speed_body_x          # [m/s] Measured speed in body x direction. Positiv: forwards, Negativ: backwards
float32 adjusted_speed_body_x_setpoint # [m/s] Post slew rate speed setpoint in body x direction. Positiv: forwards, Negativ: backwards
float32 pid_throttle_body_x_integral   # Integral of the PID for the closed loop controller of the speed in body x direction
float32 measured_speed_body_y          # [m/s] Measured speed in body y direction. Positiv: right, Negativ: left (Mecanum only)
float32 adjusted_speed_body_y_setpoint # [m/s] Post slew rate speed setpoint in body y direction. Positiv: right, Negativ: left (Mecanum only)
float32 pid_throttle_body_y_integral   # Integral of the PID for the closed loop controller of the speed in body y direction (Mecanum only)
