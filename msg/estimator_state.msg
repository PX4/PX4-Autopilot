# Estimator state

uint8 n				# number of states
float32[30] state	# estimator state
uint8[30] id		# field type (e.g. "bias" if it were "gyro bias", see MAV_FIELD)
uint8[30] sensor	# associated sensor (e.g. "gyro" if it were "gyro bias", see MAV_SENSOR_TYPE)

# TOPICS estimator_state
