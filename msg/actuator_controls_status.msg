uint16 saturation_status	# Integer bit mask indicating which axes in the control mixer are saturated

uint8 INDEX_VALID = 0           # True if the saturation status is valid
uint8 INDEX_ROLL_POS = 1        # True if a positive roll increment will increase motor saturation
uint8 INDEX_ROLL_NEG = 2        # True if negative roll increment will increase motor saturation
uint8 INDEX_PITCH_POS = 3       # True if positive pitch increment will increase motor saturation
uint8 INDEX_PITCH_NEG = 4       # True if negative pitch increment will increase motor saturation
uint8 INDEX_YAW_POS = 5         # True if positive yaw increment will increase motor saturation
uint8 INDEX_YAW_NEG = 6         # True if negative yaw increment will increase motor saturation
uint8 INDEX_X_THRUST_POS = 7    # True if positive X thrust increment will increase motor saturation
uint8 INDEX_X_THRUST_NEG = 8    # True if negative X thrust increment will increase motor saturation
uint8 INDEX_Y_THRUST_POS = 9    # True if positive Y thrust increment will increase motor saturation
uint8 INDEX_Y_THRUST_NEG = 10   # True if negative Y thrust increment will increase motor saturation
uint8 INDEX_Z_THRUST_POS = 11   # True if positive Z thrust increment will increase motor saturation
uint8 INDEX_Z_THRUST_NEG = 12   # True if negative Z thrust increment will increase motor saturation
uint8 INDEX_X_THRUST_VALID = 13 # True if X thrust is controllable. If False, X thrust commands are considered 0
uint8 INDEX_Y_THRUST_VALID = 14 # True if Y thrust is controllable. If False, Y thrust commands are considered 0
uint8 INDEX_Z_THRUST_VALID = 15 # True if Z thrust is controllable. If False, Z thrust commands are considered 0