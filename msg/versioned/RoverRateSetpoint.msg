# Rover Rate setpoint

uint32 MESSAGE_VERSION = 0

uint64  timestamp         # Time since system start [us]
float32 yaw_rate_setpoint # Yaw rate setpoint [rad/s] [@range -inf, inf] [@frame NED]
