# Rover Attitude Status

uint64  timestamp             # Time since system start [us]
float32 measured_yaw          # Measured yaw [rad] [@range -pi, pi] [@frame NED]
float32 adjusted_yaw_setpoint # Yaw setpoint that is being tracked (Applied slew rates) [rad] [@range -pi, pi] [@frame NED]
