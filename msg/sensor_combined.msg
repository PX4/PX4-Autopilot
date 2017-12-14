#
# Sensor readings in SI-unit form.
#
# These fields are scaled and offset-compensated where possible and do not
# change with board revisions and sensor updates.
#

int32 RELATIVE_TIMESTAMP_INVALID = 2147483647 # (0x7fffffff) If one of the relative timestamps is set to this value, it means the associated sensor values are invalid


# gyro timstamp is equal to the timestamp of the message
float32[3] gyro_rad			# average angular rate measured in the XYZ body frame in rad/s over the last gyro sampling period
uint32 gyro_integral_dt		# gyro measurement sampling period in us

int32 accelerometer_timestamp_relative	# timestamp + accelerometer_timestamp_relative = Accelerometer timestamp
float32[3] accelerometer_m_s2		# average value acceleration measured in the XYZ body frame in m/s/s over the last accelerometer sampling period
uint32 accelerometer_integral_dt	# accelerometer measurement sampling period in us

int32 magnetometer_timestamp_relative	# timestamp + magnetometer_timestamp_relative = Magnetometer timestamp
float32[3] magnetometer_ga		# Magnetic field in NED body frame, in Gauss

int32 baro_timestamp_relative		# timestamp + baro_timestamp_relative = Barometer timestamp
float32 baro_alt_meter			# Altitude above MSL calculated from temperature compensated baro sensor data using an ISA corrected for sea level pressure SENS_BARO_QNH.
float32 baro_temp_celcius		# Temperature in degrees celsius
float32 baro_pressure_pa		# Absolute pressure in pascals
