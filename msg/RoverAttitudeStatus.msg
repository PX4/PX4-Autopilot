# Rover Attitude Status

uint64 timestamp               # [us] Time since system start
float32 measured_yaw           # [rad] [@range -pi, pi] [@frame NED]Measured yaw
float32 adjusted_yaw_setpoint  # [rad] [@range -pi, pi] [@frame NED] Yaw setpoint that is being tracked (Applied slew rates)
