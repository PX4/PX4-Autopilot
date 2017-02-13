# Estimator innovation standard deviation

uint8 n				# number of innovations
float32[40] std		# estimator innovation standard deviation sqrt(diag(P))
uint8[40] id		# field type (e.g. "bias" if it were "gyro bias", see MAV_FIELD)
uint8[40] sensor	# associated sensor (e.g. "gyro" if it were "gyro bias", see MAV_SENSOR_TYPE)

# TOPICS estimator_innov_std
