uint64 timestamp

# NOTE: At least one of course, airspeed_reference_direction, or lateral_acceleration must be finite

float32 course 				# [-pi, pi] Course over ground setpoint. NAN if not controlled directly.
float32 airspeed_reference_direction    # [-pi, pi] Angle projected to ground of desired (true) airspeed vector. NAN if not controlled directly, used as feedforward if course setpoint is finite.
float32 lateral_acceleration 		# [m/s^2] Lateral acceleration setpoint. NAN if not controlled directly, used as feedforward if either course setpoint or airspeed_reference_direction is finite.

# TOPICS fixed_wing_lateral_setpoint fixed_wing_lateral_setpoint_status
