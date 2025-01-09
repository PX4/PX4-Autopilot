uint64 timestamp


# Note: If not both pitch_sp and throttle_sp are finite, then either altitude_setpoint or height_rate_setpoint must be finite
float32 height_rate_setpoint # NAN if not controlled directly, used as feeforward if altitude_setpoint is finite [m/s]
float32 altitude_setpoint  # NAN if not controlled, MSL [m]
float32 equivalent_airspeed_setpoint # [m/s]
float32 pitch_sp # NAN if not controlled, overrides total energy controller [rad]
float32 thrust_sp # NAN if not controlled, overrides total energy controller [0,1]

# TOPICS fw_longitudinal_control_setpoint fw_longitudinal_control_status
