# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#
uint64 time_utc_usec		# GPS UTC timestamp, (microseconds)
float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt			# Altitude AMSL, (meters)
float64[2] delta_lat_lon # Reset delta for horizontal position coordinates
float32 delta_alt 	# Reset delta for altitude
uint8 lat_lon_reset_counter	# Counter for reset events on horizontal position coordinates
uint8 alt_reset_counter 	# Counter for reset events on altitude
float32 vel_n			# North velocity in NED earth-fixed frame, (metres/sec)
float32 vel_e			# East velocity in NED earth-fixed frame, (metres/sec)
float32 vel_d			# Down velocity in NED earth-fixed frame, (metres/sec)
float32 pos_d_deriv		# Down position time derivative in NED earth-fixed frame, (metres/sec)
float32 yaw 			# Euler yaw angle relative to NED earth-fixed frame, -PI..+PI, (radians)
float32 eph			# Standard deviation of horizontal position error, (metres)
float32 epv			# Standard deviation of vertical position error, (metres)
float32 evh			# Standard deviation of horizontal velocity error, (metres/sec)
float32 evv			# Standard deviation of horizontal velocity error, (metres/sec)
float32 terrain_alt		# Terrain altitude WGS84, (metres)
bool terrain_alt_valid		# Terrain altitude estimate is valid
bool dead_reckoning		# True if this position is estimated through dead-reckoning
float32 pressure_alt		# Pressure altitude AMSL, (metres)

# TOPICS vehicle_global_position vehicle_global_position_groundtruth
