#
# Sensor readings in SI-unit form.
#
# These fields are scaled and offset-compensated where possible and do not
# change with board revisions and sensor updates.
#

uint64[3] gyro_timestamp			# Gyro timestamps
float32[9] gyro_integral_rad		# delta angle in the NED body frame in rad in the integration time frame
uint64[3] gyro_integral_dt			# delta time for gyro integral in us

float32[9] accelerometer_integral_m_s		# velocity in NED body frame, in m/s^2
uint64[3] accelerometer_integral_dt		# delta time for accel integral in us
uint64[3] accelerometer_timestamp	# Accelerometer timestamp

float32[9] magnetometer_ga		# Magnetic field in NED body frame, in Gauss
uint64[3] magnetometer_timestamp	# Magnetometer timestamp

float32[3] baro_alt_meter			# Altitude, already temp. comp.
float32[3] baro_temp_celcius		# Temperature in degrees celsius
uint64[3] baro_timestamp		# Barometer timestamp

