uint64 timestamp			# time since system start (microseconds)

uint8 id			# Onboard ID of the sensor


float32 measurement_windspeed_x_m_s		# indicated windspeed_x in m/s

float32 measurement_windspeed_y_m_s		# indicated windspeed_y in m/s

float32 measurement_windspeed_z_m_s		# indicated windspeed_z in m/s

float32 filtered_windspeed_x_m_s		# true filtered windspeed_x in m/s

float32 filtered_windspeed_y_m_s		# true filtered windspeed_y in m/s

float32 filtered_windspeed_z_m_s		# true filtered windspeed_z in m/s

float32 air_temperature_celsius		# air temperature in degrees celsius, -1000 if unknown

float32 confidence_x			# confidence_x value from 0 to 1 for this sensor

float32 confidence_y			# confidence_y value from 0 to 1 for this sensor

float32 confidence_z			# confidence_z value from 0 to 1 for this sensor


uint8 orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum

uint8 ROTATION_YAW_0		= 0 # MAV_SENSOR_ROTATION_NONE
uint8 ROTATION_YAW_45		= 1 # MAV_SENSOR_ROTATION_YAW_45
uint8 ROTATION_YAW_90		= 2 # MAV_SENSOR_ROTATION_YAW_90
uint8 ROTATION_YAW_135		= 3 # MAV_SENSOR_ROTATION_YAW_135
uint8 ROTATION_YAW_180		= 4 # MAV_SENSOR_ROTATION_YAW_180
uint8 ROTATION_YAW_225		= 5 # MAV_SENSOR_ROTATION_YAW_225
uint8 ROTATION_YAW_270		= 6 # MAV_SENSOR_ROTATION_YAW_270
uint8 ROTATION_YAW_315		= 7 # MAV_SENSOR_ROTATION_YAW_315

uint8 ROTATION_FORWARD_FACING	= 0 # MAV_SENSOR_ROTATION_NONE
uint8 ROTATION_RIGHT_FACING	= 2 # MAV_SENSOR_ROTATION_YAW_90
uint8 ROTATION_BACKWARD_FACING	= 4 # MAV_SENSOR_ROTATION_YAW_180
uint8 ROTATION_LEFT_FACING	= 6 # MAV_SENSOR_ROTATION_YAW_270

uint8 ROTATION_UPWARD_FACING   = 24 # MAV_SENSOR_ROTATION_PITCH_90
uint8 ROTATION_DOWNWARD_FACING = 25 # MAV_SENSOR_ROTATION_PITCH_270

uint8 ROTATION_CUSTOM          = 100 # MAV_SENSOR_ROTATION_CUSTOM
