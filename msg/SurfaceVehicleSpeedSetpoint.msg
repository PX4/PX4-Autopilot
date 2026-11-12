# Surface Vehicle Speed Setpoint

uint64 timestamp  # [us] Time since system start
float32 x         # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint along the body X axis
float32 y         # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint along the body Y axis
