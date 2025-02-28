uint64 timestamp

# NOTE: At least one of course_setpoint, airspeed_reference_direction, or lateral_acceleration_setpoint must be finite

float32 course_setpoint 		# [-pi, pi], NAN if not controlled directly
float32 airspeed_reference_direction    # [-pi, pi], angle of desired airspeed vector, NAN if not controlled directly, used as feedforward if course setpoint is finite
float32 lateral_acceleration_setpoint 	# [m/s^2], NAN if not controlled directly, used as feedforward if either course setpoint or airspeed_reference_direction is finite
float32 roll_sp 			# TODO: remove, only for testing

# TOPICS fw_lateral_control_setpoint fw_lateral_control_status
