# landing target GNSS position in WGS84 coordinates.

uint64 timestamp		# time since system start (microseconds)

float64 latitude_deg		# Latitude in degrees, allows centimeter level RTK precision
float64 longitude_deg		# Longitude in degrees, allows centimeter level RTK precision
float32 altitude_msl_m		# Altitude above MSL, meters

float32 eph			    # GNSS horizontal position accuracy (metres)
float32 epv			    # GNSS vertical position accuracy (metres)

bool abs_pos_updated    # True if WGS84 position is updated

float32 vel_n_m_s		# GNSS North velocity, (metres/sec)
float32 vel_e_m_s		# GNSS East velocity, (metres/sec)
float32 vel_d_m_s		# GNSS Down velocity, (metres/sec)

float32 s_acc_m_s	# GNSS speed accuracy estimate, (metres/sec)

bool vel_ned_updated    # True if NED velocity is updated
