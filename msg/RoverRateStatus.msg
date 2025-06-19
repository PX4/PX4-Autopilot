# Rover Rate Status

uint64  timestamp                  # Time since system start [us]
float32 measured_yaw_rate          # Measured yaw rate [rad/s] [@range -inf, inf] [@frame NED]
float32 adjusted_yaw_rate_setpoint # Yaw rate setpoint that is being tracked (Applied slew rates) [rad/s] [@range -inf, inf] [@frame NED]
float32 pid_yaw_rate_integral      # Integral of the PID for the closed loop yaw rate controller [] [@range -1, 1]
