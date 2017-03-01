# uint64 timestamp 			# ekf2 reference time. This is a timestamp passed to the
					# estimator which it uses a absolute reference.
float32 gyro_integral_dt		# gyro integration period in s
float32 accelerometer_integral_dt 	# accelerometer integration period in s
uint64 magnetometer_timestamp 		# timestamp of magnetometer measurement in us
uint64 baro_timestamp			# timestamp of barometer measurement in us
uint64 rng_timestamp			# timestamp of range finder measurement in us
uint64 flow_timestamp			# timestamp of optical flow measurement in us
uint64 asp_timestamp			# timestamp of airspeed measurement in us
uint64 ev_timestamp			# timestamp of external vision measurement in us

float32[3] gyro_rad			# gyro vector in rad
float32[3] accelerometer_m_s2  	# accelerometer vector in m/s^2

float32[3] magnetometer_ga 		# magnetometer measurement vector (body fixed NED) in ga
float32 baro_alt_meter			# barometer altitude measurement in m

uint64 time_usec			# timestamp of gps position measurement in us
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

# airspeed
float32 indicated_airspeed_m_s		# indicated airspeed in meters per second, -1 if unknown
float32 true_airspeed_m_s		# true filtered airspeed in meters per second, -1 if unknown

# external vision measurements
float32[3] pos_ev			# position in earth (NED) frame (metres)
float32[4] quat_ev			# quaternion rotation from earth (NED) to body (XYZ) frame
float32 pos_err				# position error 1-std for each axis (metres)
float32 ang_err				# angular error 1-std for each axis (rad)
