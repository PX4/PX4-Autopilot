# Fused external local position in NED.
# The coordinate system origin is the vehicle position at the time when the EKF2-module was started.

uint32 MESSAGE_VERSION = 0

uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)

bool xy_valid				# true if x and y are valid
bool z_valid				# true if z is valid
bool v_xy_valid				# true if vx and vy are valid
bool v_z_valid				# true if vz is valid

# Position in local NED frame
float32[3] position         # Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown

# Position reset delta
float32[2] delta_xy			# Amount of lateral shift of position estimate in latest reset (in x and y) [m]
uint8 xy_reset_counter			# Index of latest lateral position estimate reset
float32 delta_z				# Amount of vertical shift of position estimate in latest reset [m]
uint8 z_reset_counter			# Index of latest vertical position estimate reset

# Velocity in NED frame
float32[3] velocity         # Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown
float32 z_deriv				# Down position time derivative in NED earth-fixed frame, (metres/sec)

# Velocity reset delta
float32[2] delta_vxy			# Amount of lateral shift of velocity estimate in latest reset (in x and y) [m/s]
uint8 vxy_reset_counter			# Index of latest vertical velocity estimate reset
float32 delta_vz			# Amount of vertical shift of velocity estimate in latest reset [m/s]
uint8 vz_reset_counter			# Index of latest vertical velocity estimate reset

# Acceleration in NED frame
float32[3] acceleration

#Heading
float32[4] heading

uint8 heading_reset_counter		# Index of latest heading reset
bool heading_good_for_control

float32 tilt_var

# Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
bool xy_global				# true if position (x, y) has a valid global reference (ref_lat, ref_lon)
bool z_global				# true if z has a valid global reference (ref_alt)
uint64 ref_timestamp			# Time when reference position was set since system start, (microseconds)
float64 ref_lat				# Reference point latitude, (degrees)
float64 ref_lon				# Reference point longitude, (degrees)
float32 ref_alt				# Reference altitude AMSL, (metres)

# Distance to surface
bool dist_bottom_valid			# true if distance to bottom surface is valid
float32 dist_bottom			# Distance from from bottom surface to ground, (metres)
float32 dist_bottom_var                 # terrain estimate variance (m^2)

float32 delta_dist_bottom               # Amount of vertical shift of dist bottom estimate in latest reset [m]
uint8 dist_bottom_reset_counter         # Index of latest dist bottom estimate reset

uint8 dist_bottom_sensor_bitfield	# bitfield indicating what type of sensor is used to estimate dist_bottom
uint8 DIST_BOTTOM_SENSOR_NONE = 0
uint8 DIST_BOTTOM_SENSOR_RANGE = 1	# (1 << 0) a range sensor is used to estimate dist_bottom field
uint8 DIST_BOTTOM_SENSOR_FLOW = 2	# (1 << 1) a flow sensor is used to estimate dist_bottom field (mostly fixed-wing use case)

float32 eph				# Standard deviation of horizontal position error, (metres)
float32 epv				# Standard deviation of vertical position error, (metres)
float32 evh				# Standard deviation of horizontal velocity error, (metres/sec)
float32 evv				# Standard deviation of vertical velocity error, (metres/sec)

bool dead_reckoning                     # True if this position is estimated through dead-reckoning

# estimator specified vehicle limits
# set to INFINITY when limiting not required
float32 vxy_max				# maximum horizontal speed (meters/sec)
float32 vz_max				# maximum vertical speed (meters/sec)
float32 hagl_min			# minimum height above ground level (meters)
float32 hagl_max_z			# maximum height above ground level for z-control (meters)
float32 hagl_max_xy			# maximum height above ground level for xy-control (meters)

#TOPICS external_vehicle_local_position
