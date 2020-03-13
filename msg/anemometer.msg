# ANEMOMETER_SENSOR message data

uint64 timestamp		# time since system start (microseconds)

uint64 error_count		# Number of errors detected by driver

float32 current_speed		# Current airspeed reading (in m)
float32 variance		# Measurement variance (in m^2/s^2), 0 for unknown / invalid readings
int8 signal_quality		# Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.

uint8 id			# Onboard ID of the sensor

uint8 orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum
uint8 ROTATION_DOWNWARD_FACING = 5 # MAV_SENSOR_ROTATION_PITCH_270
uint8 ROTATION_UPWARD_FACING   = 4 # MAV_SENSOR_ROTATION_PITCH_90
uint8 ROTATION_BACKWARD_FACING = 2 # MAV_SENSOR_ROTATION_PITCH_180
uint8 ROTATION_FORWARD_FACING  = 0  # MAV_SENSOR_ROTATION_NONE
uint8 ROTATION_LEFT_FACING     = 3  # MAV_SENSOR_ROTATION_YAW_270
uint8 ROTATION_RIGHT_FACING    = 1  # MAV_SENSOR_ROTATION_YAW_90

