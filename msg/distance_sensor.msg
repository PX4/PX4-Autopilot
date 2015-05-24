# DISTANCE_SENSOR message data

uint64 timestamp		# Microseconds since system boot

float32 min_distance		# Minimum distance the sensor can measure (in m)
float32 max_distance		# Maximum distance the sensor can measure (in m)
float32 current_distance	# Current distance reading (in m)
float32 covariance		# Measurement covariance (in m), 0 for unknown / invalid readings

uint8 type			# Type from MAV_DISTANCE_SENSOR enum
uint8 MAV_DISTANCE_SENSOR_LASER = 0
uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1
uint8 MAV_DISTANCE_SENSOR_INFRARED = 2

uint8 id			# Onboard ID of the sensor

uint8 orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum
