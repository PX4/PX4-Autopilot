uint64 timestamp		# time since system start (microseconds)
uint16 saturation_status	# Integer bit mask indicating which axes in the control mixer are saturated

uint8 INDEX_VALID = 0           # True if the saturation status is valid
uint8 INDEX_MOTOR_POS = 1       # True if any motor is saturated at the upper limit
uint8 INDEX_MOTOR_NEG = 2       # True if any motor is saturated at the lower limit
uint8 INDEX_ROLL_POS = 3        # True if a positive roll increment will increase motor saturation
uint8 INDEX_ROLL_NEG = 4        # True if negative roll increment will increase motor saturation
uint8 INDEX_PITCH_POS = 5       # True if positive pitch increment will increase motor saturation
uint8 INDEX_PITCH_NEG = 6       # True if negative pitch increment will increase motor saturation
uint8 INDEX_YAW_POS = 7         # True if positive yaw increment will increase motor saturation
uint8 INDEX_YAW_NEG = 8         # True if negative yaw increment will increase motor saturation
uint8 INDEX_X_THRUST_POS = 9    # True if positive X thrust increment will increase motor saturation
uint8 INDEX_X_THRUST_NEG = 10   # True if negative X thrust increment will increase motor saturation
uint8 INDEX_Y_THRUST_POS = 11   # True if positive Y thrust increment will increase motor saturation
uint8 INDEX_Y_THRUST_NEG = 12   # True if negative Y thrust increment will increase motor saturation
uint8 INDEX_Z_THRUST_POS = 13   # True if positive Z thrust increment will increase motor saturation
uint8 INDEX_Z_THRUST_NEG = 14   # True if negative Z thrust increment will increase motor saturation
