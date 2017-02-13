# Estimator innovation

uint8 n				# number of innovations
float32[40] innov	# estimator innovation
uint8[40] id		# field type (e.g. "bias" if it were "gyro bias", see MAV_FIELD)
uint8[40] sensor	# associated sensor (e.g. "gyro" if it were "gyro bias", see MAV_SENSOR_TYPE)

# TOPICS estimator_innov
