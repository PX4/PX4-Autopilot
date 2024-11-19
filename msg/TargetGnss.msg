# landing target GPS position in WGS84 coordinates.

uint64 timestamp		# time since system start (microseconds)

uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
uint8 satellites_used		# Number of satellites used

float64 latitude_deg		# Latitude in degrees, allows centimeter level RTK precision
float64 longitude_deg		# Longitude in degrees, allows centimeter level RTK precision
float32 altitude_msl_m		# Altitude above MSL, meters

float32 eph			    # GPS horizontal position accuracy (metres)
float32 epv			    # GPS vertical position accuracy (metres)

bool abs_pos_updated    # True if WGS84 position is updated

float32 vel_n_m_s		# GPS North velocity, (metres/sec)
float32 vel_e_m_s		# GPS East velocity, (metres/sec)
float32 vel_d_m_s		# GPS Down velocity, (metres/sec)

float32 s_variance_m_s	# GPS speed accuracy estimate, (metres/sec)

bool vel_ned_updated    # True if NED velocity is updated
