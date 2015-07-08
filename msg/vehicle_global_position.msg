# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#
uint64 timestamp		# Time of this estimate, in microseconds since system start
uint64 time_utc_usec		# GPS UTC timestamp in microseconds
float64 lat			# Latitude in degrees
float64 lon			# Longitude in degrees
float32 alt			# Altitude AMSL in meters
float32 vel_n			# Ground north velocity, m/s
float32 vel_e			# Ground east velocity, m/s
float32 vel_d			# Ground downside velocity, m/s
float32 yaw 			# Yaw in radians -PI..+PI.
float32 eph			# Standard deviation of position estimate horizontally
float32 epv			# Standard deviation of position vertically
float32 terrain_alt		# Terrain altitude in m, WGS84
bool terrain_alt_valid		# Terrain altitude estimate is valid
bool dead_reckoning		# True if this position is estimated through dead-reckoning
