# Estimator standard deviation state

uint8 n				# number of states
float32[21] std		# estimator state
uint8[21] id		# field type (e.g. "bias" if it were "gyro bias", see MAV_FIELD)
uint8[21] sensor	# associated sensor (e.g. "gyro" if it were "gyro bias", see MAV_SENSOR_TYPE)

# TOPICS estimator_state_std
