# Rover Throttle setpoint

uint32 MESSAGE_VERSION = 0

uint64  timestamp       # Time since system start [us]
float32 throttle_body_x # Throttle setpoint along body X axis [] [@range -1 (Backwards), 1 (Forwards)] [@frame Body]
float32 throttle_body_y # Mecanum only: Throttle setpoint along body Y axis [] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not mecanum]
