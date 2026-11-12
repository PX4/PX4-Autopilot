# Surface Vehicle Throttle Setpoint

uint64 timestamp  # [us] Time since system start
float32 x         # [-] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Normalized throttle setpoint along the body X axis
float32 y         # [-] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Normalized throttle setpoint along the body Y axis
