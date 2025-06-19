# Rover attitude setpoint

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
float32 yaw_setpoint # [rad] [@range -inf, inf] [@frame NED] # Yaw setpoint
