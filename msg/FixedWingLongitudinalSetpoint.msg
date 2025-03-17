uint64 timestamp

# Note: If not both pitch_direct and thrust_direct are finite, then either altitude or height_rate must be finite

float32 altitude  			# [m] Altitdue setpoint AMSL, NAN if not controlled
float32 height_rate 			# [m/s] NAN if not controlled directly, used as feedforward if altitude is finite
float32 equivalent_airspeed 		# [m/s] NAN if system default should be used
float32 pitch_direct 			# [rad] NAN if not controlled, overrides total energy controller
float32 thrust_direct 			# [0,1] NAN if not controlled, overrides total energy controller

# TOPICS fixed_wing_longitudinal_setpoint fixed_wing_longitudinal_setpoint_status
