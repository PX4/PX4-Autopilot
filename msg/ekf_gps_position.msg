# EKF blended position in WGS84 coordinates.
uint64 timestamp		# time since system start (microseconds)
int32 lat			# Latitude in 1E-7 degrees
int32 lon			# Longitude in 1E-7 degrees
int32 alt			# Altitude in 1E-3 meters above MSL, (millimetres)
int32 alt_ellipsoid 		# Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
float32 s_variance_m_s		# GPS speed accuracy estimate, (metres/sec)
uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
float32 eph			# GPS horizontal position accuracy (metres)
float32 epv			# GPS vertical position accuracy (metres)
float32 vel_m_s			# GPS ground speed, (metres/sec)
float32 vel_n_m_s		# GPS North velocity, (metres/sec)
float32 vel_e_m_s		# GPS East velocity, (metres/sec)
float32 vel_d_m_s		# GPS Down velocity, (metres/sec)
bool vel_ned_valid		# True if NED velocity is valid
uint8 satellites_used		# Number of satellites used
uint8 selected			# GPS selection: 0: GPS1, 1: GPS2. 2: GPS1+GPS2 blend
