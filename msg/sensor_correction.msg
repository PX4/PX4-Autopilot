#
# Sensor corrections in SI-unit form for the voted sensor
#

# Corrections for gyro angular rate outputs where corrected_rate = raw_rate * gyro_scale + gyro_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
float32[3] gyro_offset		# gyro XYZ offsets in the sensor frame in rad/s
float32[3] gyro_scale		# gyro XYZ scale factors in the sensor frame

# Corrections for acceleromter acceleration outputs where corrected_accel = raw_accel * accel_scale + accel_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
float32[3] accel_offset		# accelerometer XYZ offsets in the sensor frame in m/s/s
float32[3] accel_scale		# accelerometer XYZ scale factors in the sensor frame

# Corrections for barometric pressure outputs where corrected_pressure = raw_pressure * pressure_scale + pressure_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
float32 baro_offset		# barometric pressure offsets in the sensor frame in m/s/s
float32 baro_scale		# barometric pressure scale factors in the sensor frame

uint8 selected_gyro_instance		# gyro uORB topic instance for the voted sensor
uint8 selected_accel_instance		# accelerometer uORB topic instance for the voted sensor
uint8 selected_baro_instance		# barometric pressure uORB topic instance for the voted sensor
