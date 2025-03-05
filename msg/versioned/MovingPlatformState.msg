# Position and velocity of moving platform to land on (ship, truck, train, ...)

uint32 MESSAGE_VERSION = 0

uint64 timestamp			# time since system start (microseconds)

# Position in WGS84
float64 lat				# Latitude in degrees
float64 lon				# Longitude in degrees
float32 alt				# Altitude in meters (AMSL)

# Velocity in local NED tangent frame
float32 v_north			# Velocity in north direction in m/s
float32 v_east			# Velocity in east direction in m/s
float32 v_down			# Velocity in down direction in m/s

# TOPICS moving_platform_state
