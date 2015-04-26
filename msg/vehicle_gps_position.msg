# GPS position in WGS84 coordinates.
uint64 timestamp_position	# Timestamp for position information 
int32 lat			# Latitude in 1E-7 degrees 
int32 lon			# Longitude in 1E-7 degrees 
int32 alt			# Altitude in 1E-3 meters (millimeters) above MSL  

uint64 timestamp_variance
float32 s_variance_m_s		# speed accuracy estimate m/s 
float32 c_variance_rad		# course accuracy estimate rad 
uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.   

float32 eph			# GPS HDOP horizontal dilution of position in m 
float32 epv			# GPS VDOP horizontal dilution of position in m 

int32 noise_per_ms		# GPS noise per millisecond
int32 jamming_indicator		# indicates jamming is occurring

uint64 timestamp_velocity	# Timestamp for velocity informations 
float32 vel_m_s			# GPS ground speed (m/s) 
float32 vel_n_m_s		# GPS ground speed in m/s 
float32 vel_e_m_s		# GPS ground speed in m/s 
float32 vel_d_m_s		# GPS ground speed in m/s 
float32 cog_rad			# Course over ground (NOT heading, but direction of movement) in rad, -PI..PI 
bool vel_ned_valid		# Flag to indicate if NED speed is valid 

uint64 timestamp_time		# Timestamp for time information 
uint64 time_utc_usec		# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0 

uint8 satellites_used		# Number of satellites used 
