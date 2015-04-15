# Fused local position in NED.

uint64 timestamp		# Time of this estimate, in microseconds since system start
bool xy_valid			# true if x and y are valid
bool z_valid			# true if z is valid
bool v_xy_valid		# true if vy and vy are valid
bool v_z_valid			# true if vz is valid

# Position in local NED frame
float32 x				# X position in meters in NED earth-fixed frame
float32 y				# X position in meters in NED earth-fixed frame
float32 z				# Z position in meters in NED earth-fixed frame (negative altitude)

# Velocity in NED frame
float32 vx 				# Ground X Speed (Latitude), m/s in NED
float32 vy				# Ground Y Speed (Longitude), m/s in NED
float32 vz				# Ground Z Speed (Altitude), m/s	in NED

# Heading
float32 yaw

# Reference position in GPS / WGS84 frame
bool xy_global			# true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
bool z_global			# true if z is valid and has valid global reference (ref_alt)
uint64 ref_timestamp	# Time when reference position was set
float64 ref_lat		# Reference point latitude in degrees
float64 ref_lon		# Reference point longitude in degrees
float32 ref_alt			# Reference altitude AMSL in meters, MUST be set to current (not at reference point!) ground level

# Distance to surface
float32 dist_bottom		# Distance to bottom surface (ground)
float32 dist_bottom_rate		# Distance to bottom surface (ground) change rate
uint64 surface_bottom_timestamp		# Time when new bottom surface found
bool dist_bottom_valid	# true if distance to bottom surface is valid
float32 eph
float32 epv
