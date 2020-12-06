#
# Sensor corrections in SI-unit form for the voted sensor
#

uint64 timestamp		# time since system start (microseconds)

# Corrections for gyro angular rate outputs where corrected_rate = raw_rate * gyro_scale + gyro_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
uint32[4] gyro_device_ids
float32[3] gyro_offset_0	# gyro 0 XYZ offsets in the sensor frame in rad/s
float32[3] gyro_offset_1	# gyro 1 XYZ offsets in the sensor frame in rad/s
float32[3] gyro_offset_2	# gyro 2 XYZ offsets in the sensor frame in rad/s
float32[3] gyro_offset_3	# gyro 3 XYZ offsets in the sensor frame in rad/s

# Corrections for acceleromter acceleration outputs where corrected_accel = raw_accel * accel_scale + accel_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
<<<<<<< HEAD
uint32[4] accel_device_ids
float32[3] accel_offset_0	# accelerometer 0 XYZ offsets in the sensor frame in m/s/s
float32[3] accel_offset_1	# accelerometer 1 XYZ offsets in the sensor frame in m/s/s
float32[3] accel_offset_2	# accelerometer 2 XYZ offsets in the sensor frame in m/s/s
float32[3] accel_offset_3	# accelerometer 3 XYZ offsets in the sensor frame in m/s/s

# Corrections for barometric pressure outputs where corrected_pressure = raw_pressure * pressure_scale + pressure_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
uint32[4] baro_device_ids
float32 baro_offset_0		# barometric pressure 0 offsets in the sensor frame in m/s/s
float32 baro_offset_1		# barometric pressure 1 offsets in the sensor frame in m/s/s
float32 baro_offset_2		# barometric pressure 2 offsets in the sensor frame in m/s/s
float32 baro_offset_3		# barometric pressure 3 offsets in the sensor frame in m/s/s
=======
# corrections for uORB instance 0
float32[3] accel_offset_0	# accelerometer XYZ offsets in the sensor frame in m/s^2
float32[3] accel_scale_0	# accelerometer XYZ scale factors in the sensor frame

# corrections for uORB instance 1
float32[3] accel_offset_1	# accelerometer XYZ offsets in the sensor frame in m/s^2
float32[3] accel_scale_1	# accelerometer XYZ scale factors in the sensor frame

# corrections for uORB instance 2
float32[3] accel_offset_2	# accelerometer XYZ offsets in the sensor frame in m/s^2
float32[3] accel_scale_2	# accelerometer XYZ scale factors in the sensor frame

# Corrections for barometric pressure outputs where corrected_pressure = raw_pressure * pressure_scale + pressure_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
# corrections for uORB instance 0
float32 baro_offset_0		# barometric pressure offsets in the sensor frame in m/s^2
float32 baro_scale_0		# barometric pressure scale factors in the sensor frame

# corrections for uORB instance 1
float32 baro_offset_1		# barometric pressure offsets in the sensor frame in m/s^2
float32 baro_scale_1		# barometric pressure scale factors in the sensor frame

# corrections for uORB instance 2
float32 baro_offset_2		# barometric pressure offsets in the sensor frame in m/s^2
float32 baro_scale_2		# barometric pressure scale factors in the sensor frame

# Mapping from uORB index to parameter index for each sensor type. For example accel_mapping[1] contains the
# compensation parameter system index for the sensor_accel uORB index 1 data.
uint8[3] gyro_mapping
uint8[3] accel_mapping
uint8[3] baro_mapping

uint32[3] gyro_device_ids
uint32[3] accel_device_ids
uint32[3] baro_device_ids
>>>>>>> ebdcde2ca02aa89933daecd7a1d0cbb87a33dc00
