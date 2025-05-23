# Fixed Wing Longitudinal Setpoint message
# Used by the fw_lateral_longitudinal_control module
# If pitch_direct and throttle_direct are not both finite, then the controller relies on altitude/height_rate and equivalent_airspeed to control vertical motion.
# If both altitude and height_rate are NAN, the controller maintains the current altitude.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                        # time since system start (microseconds)

float32 altitude  			# [m] Altitude setpoint AMSL, not controlled directly if NAN or if height_rate is finite
float32 height_rate 			# [m/s] [ENU] Scalar height rate setpoint. NAN if not controlled directly
float32 equivalent_airspeed 		# [m/s] [@range 0, inf] Scalar equivalent airspeed setpoint. NAN if system default should be used
float32 pitch_direct 			# [rad] [@range -pi, pi] [FRD] NAN if not controlled, overrides total energy controller
float32 throttle_direct 		# [norm] [@range 0,1] NAN if not controlled, overrides total energy controller
