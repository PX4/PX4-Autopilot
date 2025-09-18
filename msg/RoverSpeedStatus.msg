# Rover Velocity Status

uint64 timestamp                        # [us] Time since system start
float32 measured_speed_body_x           # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Measured speed in body x direction
float32 adjusted_speed_body_x_setpoint  # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint in body x direction that is being tracked (Applied slew rates)
float32 pid_throttle_body_x_integral    # [-] [@range -1, 1] Integral of the PID for the closed loop controller of the speed in body x direction
float32 measured_speed_body_y           # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Measured speed in body y direction
float32 adjusted_speed_body_y_setpoint  # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint in body y direction that is being tracked (Applied slew rates)
float32 pid_throttle_body_y_integral    # [-] [@range -1, 1] [@invalid NaN If not mecanum] Mecanum only: Integral of the PID for the closed loop controller of the speed in body y direction
