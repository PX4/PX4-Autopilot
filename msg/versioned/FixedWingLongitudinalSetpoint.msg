# Fixed Wing Longitudinal Setpoint message
# Used by the fw_lateral_longitudinal_control module
# If not both pitch_direct and throttle_direct are finite, then either altitude or height_rate must be finite.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                        # time since system start (microseconds)

float32 altitude  			# [m] Altitude setpoint AMSL, not controlled directly if NAN or if height_rate is finite
float32 height_rate 			# [m/s] NAN if not controlled directly
float32 equivalent_airspeed 		# [m/s] NAN if system default should be used
float32 pitch_direct 			# [rad] [FRD] NAN if not controlled, overrides total energy controller
float32 throttle_direct 		# [0,1] NAN if not controlled, overrides total energy controller
