# Rover Speed Setpoint

uint64 timestamp      # [us] Time since system start
float32 speed_body_x  # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint in body x direction
float32 speed_body_y  # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint in body y direction
