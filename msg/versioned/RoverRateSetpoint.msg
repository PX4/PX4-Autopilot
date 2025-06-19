# Rover rate setpoint

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
float32 yaw_rate_setpoint # [rad/s] [@range -inf, inf] [@frame NED] Yaw rate setpoint
