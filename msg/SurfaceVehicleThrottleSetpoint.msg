# Surface Vehicle Throttle setpoint

uint64 timestamp # [us] Time since system start
float32 throttle_body_x # [] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Throttle setpoint along body X axis
float32 throttle_body_y # [] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not omnidirectional] Omnidirectional only: Throttle setpoint along body Y axis
