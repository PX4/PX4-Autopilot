# Fixed Wing Lateral Setpoint message
# Used by the fw_lateral_longitudinal_control module
# At least one of course, airspeed_direction, or lateral_acceleration must be finite.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                        # time since system start (microseconds)

float32 course 				# [rad] [@range -pi, pi] Desired direction of travel over ground w.r.t (true) North. NAN if not controlled directly.
float32 airspeed_direction    		# [rad] [@range -pi, pi] Desired horizontal angle of airspeed vector w.r.t. (true) North. Same as vehicle heading if in the absence of sideslip. NAN if not controlled directly, takes precedence over course if finite.
float32 lateral_acceleration 		# [m/s^2] [FRD] Lateral acceleration setpoint. NAN if not controlled directly, used as feedforward if either course setpoint or airspeed_direction is finite.
