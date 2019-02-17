# DISTANCE_SENSOR message data

uint64 timestamp		# time since system start (microseconds)

float32 min_distance		# Minimum distance the sensor can measure (in m)
float32 max_distance		# Maximum distance the sensor can measure (in m)
float32 current_distance	# Current distance reading (in m)
float32 covariance		# Measurement covariance (in m), 0 for unknown / invalid readings
int8 signal_quality		# Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.

uint8 type			# Type from MAV_DISTANCE_SENSOR enum
uint8 MAV_DISTANCE_SENSOR_LASER = 0
uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1
uint8 MAV_DISTANCE_SENSOR_INFRARED = 2
uint8 MAV_DISTANCE_SENSOR_RADAR = 3

uint8 id			# Onboard ID of the sensor

uint8 orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum
uint8 ROTATION_DOWNWARD_FACING = 25 # MAV_SENSOR_ROTATION_PITCH_270
uint8 ROTATION_UPWARD_FACING   = 24 # MAV_SENSOR_ROTATION_PITCH_90
uint8 ROTATION_BACKWARD_FACING = 12 # MAV_SENSOR_ROTATION_PITCH_180
uint8 ROTATION_FORWARD_FACING  = 0  # MAV_SENSOR_ROTATION_NONE
uint8 ROTATION_LEFT_FACING     = 6  # MAV_SENSOR_ROTATION_YAW_270
uint8 ROTATION_RIGHT_FACING    = 2  # MAV_SENSOR_ROTATION_YAW_90
