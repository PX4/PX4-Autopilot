uint64 time_ref 			# ekf2 reference time. This is a timestamp passed to the
					# estimator which it uses a absolute reference.
uint64 gyro_integral_dt			# gyro integration period in us
uint64 accelerometer_integral_dt 	# accelerometer integration period in us
uint64 magnetometer_timestamp 		# timestamp of magnetometer measurement in us
uint64 baro_timestamp			# timestamp of barometer measurement in us
uint64 rng_timestamp			# timestamp of range finder measurement in us
uint64 flow_timestamp			# timestamp of optical flow measurement in us

float32[3] gyro_integral_rad		# integrated gyro vector in rad
float32[3] accelerometer_integral_m_s  	# integrated accelerometer vector in m/s

float32[3] magnetometer_ga 		# magnetometer measurement vector (body fixed NED) in ga
float32 baro_alt_meter			# barometer altitude measurement in m

uint64 time_usec			# timestamp of gps position measurement in us
uint64 time_usec_vel			# timestamp of gps velocity measurement in us
int32 lat				# Latitude in 1E-7 degrees
int32 lon				# Longitude in 1E-7 degrees
int32 alt				# Altitude in 1E-3 meters above MSL, (millimetres)
uint8 fix_type
uint8 nsats				# number of GPS satellites used by the receiver
float32 eph				# GPS horizontal position accuracy (metres)
float32 epv				# GPS vertical position accuracy (metres)
float32 sacc				# GPS speed accuracy (metres/sec)
float32 vel_m_s				# GPS ground speed, (metres/sec)
float32 vel_n_m_s			# GPS North velocity, (metres/sec)
float32 vel_e_m_s			# GPS East velocity, (metres/sec)
float32 vel_d_m_s			# GPS Down velocity, (metres/sec)
bool vel_ned_valid			# True if NED velocity is valid

# range finder measurements
float32 range_to_ground			# range finder measured range to ground (m)

# optical flow sensor measurements
float32[2] flow_pixel_integral		# integrated optical flow rate around x and y axes (rad)
float32[2] flow_gyro_integral		# integrated gyro rate around x and y axes (rad)
uint32 flow_time_integral		# integration timespan (usec)
uint8 flow_quality			# Quality of accumulated optical flow data (0 - 255)
