# Rover Attitude Setpoint

uint32 MESSAGE_VERSION = 0

uint64  timestamp    # Time since system start [us]
float32 yaw_setpoint # Yaw setpoint [rad] [@range -inf, inf] [@frame NED]
