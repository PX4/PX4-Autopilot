uint64 timestamp                        	# time since system start (microseconds)

uint8 sees_desired_control_source		# desired type of manual control source, RC (1) or Mav (2)
uint8 valid_mavlink_setpoint_count		# number of Mavlink connections providing setpoints (implying joystick connected). Should be < 2.
uint8 valid_rc_setpoint_count			# number of RC connections providing setpoints. Should be < 2.

# TOPICS sees_manual_control_data
