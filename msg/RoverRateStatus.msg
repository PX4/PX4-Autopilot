# Rover Rate Status

uint64 timestamp                    # [us] Time since system start
float32 measured_yaw_rate           # [rad/s] [@range -inf, inf] [@frame NED] Measured yaw rate
float32 adjusted_yaw_rate_setpoint  # [rad/s] [@range -inf, inf] [@frame NED] Yaw rate setpoint that is being tracked (Applied slew rates)
float32 pid_yaw_rate_integral       # [-] [@range -1, 1] Integral of the PID for the closed loop yaw rate controller
