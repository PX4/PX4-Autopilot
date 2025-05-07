# Rover Velocity Status

uint64  timestamp                      # Time since system start [us]
float32 measured_speed_body_x          # Measured speed in body x direction [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body]
float32 adjusted_speed_body_x_setpoint # Speed setpoint in body x direction that is being tracked (Applied slew rates) [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body]
float32 pid_throttle_body_x_integral   # Integral of the PID for the closed loop controller of the speed in body x direction [] [@range -1, 1]
float32 measured_speed_body_y          # Mecanum only: Measured speed in body y direction [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum]
float32 adjusted_speed_body_y_setpoint # Mecanum only: Speed setpoint in body y direction that is being tracked (Applied slew rates) [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum]
float32 pid_throttle_body_y_integral   # Mecanum only: Integral of the PID for the closed loop controller of the speed in body y direction [] [@range -1, 1] [@invalid NaN If not mecanum]
