# Fused local position in NED.

bool xy_valid				# true if x and y are valid
bool z_valid				# true if z is valid
bool v_xy_valid				# true if vy and vy are valid
bool v_z_valid				# true if vz is valid

# Position in local NED frame
float32 x				# North position in NED earth-fixed frame, (metres)
float32 y				# East position in NED earth-fixed frame, (metres)
float32 z				# Down position (negative altitude) in NED earth-fixed frame, (metres)

# Position reset delta
float32[2] delta_xy
float32 delta_z

# Velocity in NED frame
float32 vx 				# North velocity in NED earth-fixed frame, (metres/sec)
float32 vy				# East velocity in NED earth-fixed frame, (metres/sec)
float32 vz				# Down velocity in NED earth-fixed frame, (metres/sec)

# Velocity reset delta
float32[2] delta_vxy
float32 delta_vz

uint8 xy_reset_counter
uint8 z_reset_counter
uint8 vxy_reset_counter
uint8 vz_reset_counter

# Heading
float32 yaw				# Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians) 

# Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
bool xy_global				# true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
bool z_global				# true if z is valid and has valid global reference (ref_alt)
uint64 ref_timestamp			# Time when reference position was set since system start, (microseconds)
float64 ref_lat				# Reference point latitude, (degrees)
float64 ref_lon				# Reference point longitude, (degrees)
float32 ref_alt				# Reference altitude AMSL, MUST be set to current (not at reference point!) ground level, (metres)

# Distance to surface
float32 dist_bottom			# Distance from from bottom surface to ground, (metres)
float32 dist_bottom_rate		# Rate of change of distance from bottom surface to ground, (metres/sec)
uint64 surface_bottom_timestamp		# Time when new bottom surface found since system start, (microseconds)
bool dist_bottom_valid			# true if distance to bottom surface is valid
float32 eph				# Standard deviation of horizontal position error, (metres)
float32 epv				# Standard deviation of vertical position error, (metres)

# TOPICS vehicle_local_position vehicle_local_position_groundtruth
