#
# Sensor corrections in SI-unit form for the voted sensor
#

# Corrections for gyro angular rate outputs where corrected_rate = raw_rate * gyro_scale + gyro_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
uint8 gyro_select		# gyro uORB index for the voted sensor
float32[3] gyro_offset		# gyro XYZ offsets in the sensor frame in rad/s
float32[3] gyro_scale		# gyro XYZ scale factors in the sensor frame

# Corrections for acceleromter acceleration outputs where corrected_accel = raw_aaccel * accel_scale + accel_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
uint8 accel_select		# accelerometer uORB index for the voted sensor
float32[3] accel_offset		# accelerometer XYZ offsets in the sensor frame in m/s/s
float32[3] accel_scale		# accelerometer XYZ scale factors in the sensor frame
