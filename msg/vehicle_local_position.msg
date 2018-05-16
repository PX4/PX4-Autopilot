# Vehicle odometry data. Fits ROS REP 147 for aerial vehicles

# Position in NED earth-fixed frame (meters). NaN if invalid/unknown
float32 x			# North position
float32 y			# East position
float32 z			# Down position
float32 dist_bottom		# Distance from from bottom surface to ground

# Orientation quaternion. NaN if invalid/unknown
float32[4] q			# Quaternion rotation from NED earth-fixed frame to XYZ body frame

# Row-major representation of 6x6 pose cross-covariance matrix URT.
# NED earth-fixed frame.
# Order: x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
# If invalid/unknown, first cell is NaN
float32[21] pose_covariance

# Velocity in NED earth-fixed frame (meters/sec). NaN if invalid/unknown
float32 vx 			# North velocity
float32 vy			# East velocity
float32 vz			# Down velocity
float32 z_deriv		# Down position time derivative
float32 dist_bottom_rate	# Rate of change of distance from bottom surface to ground

# Angular rate in body-fixed frame (rad/s). NaN if invalid/unknown
float32 rollspeed		# Bias corrected angular velocity about X body axis
float32 pitchspeed		# Bias corrected angular velocity about Y body axis
float32 yawspeed		# Bias corrected angular velocity about Z body axis

# Row-major representation of 6x6 velocity cross-covariance matrix URT.
# Linear velocity in NED earth-fixed frame. Angular velocity in body-fixed frame.
# Order: vx, vy, vz, rotation rate about X axis, rotation rate about Y axis, rotation rate about Z axis
# If invalid/unknown, first cell is NaN
float32[21] velocity_covariance

# Linear Acceleration in NED earth-fixed frame (metres/sec^2). NaN if invalid/unknown
float32 ax        		# North velocity derivative
float32 ay        		# East velocity derivative
float32 az        		# Down velocity derivative

# Angular Acceleration in body-fixed frame (rad/sec^2). NaN if invalid/unknown
float32 rollaccel        	# Bias corrected angular velocity derivative about X body axis
float32 pitchaccel     	# Bias corrected angular velocity derivative about X body axis
float32 yawaccel        	# Bias corrected angular velocity derivative about X body axis

# Row-major representation of 6x6 accel cross-covariance matrix URT.
# Linear acceleration in NED earth-fixed frame. Angular acceleration in body-fixed frame.
# Order: ax, ay, az, rotation accel about X axis, rotation accel about Y axis, rotation accel about Z axis
# If invalid/unknown, first cell is NaN
float32[21] accel_covariance

# Reset deltas
float32[2] delta_xy		# Amount by which X and Y position have changed during last reset
float32[2] delta_vxy		# Amount by which X and Y velocities have changed during last reset
float32 delta_z		# Amount by which Z position has changed during last reset
float32 delta_vz		# Amount by which Z velocity has changed during last reset
float32[4] delta_q 		# Amount by which quaternion has changed during last reset

# Reset delta counters
uint8 xy_reset_counter		# X and Y position reset counter
uint8 vxy_reset_counter	# X and Y velocity reset counter
uint8 z_reset_counter		# Z position reset counter
uint8 vz_reset_counter		# Z velocity reset counter
uint8 quat_reset_counter	# Quaternion reset counter

# Standard deviations. NAN if unknown
float32 eph			# Standard deviation of horizontal position error, (metres)
float32 epv			# Standard deviation of vertical position error, (metres)
float32 evh			# Standard deviation of horizontal velocity error, (metres/sec)
float32 evv			# Standard deviation of horizontal velocity error, (metres/sec)
float32 att_std_dev		# Standard deviation of attitude error, (radians)
float32 att_rate_std_dev	# Standard deviation of attitude rate error, (radians/sec)

# Validation flags
# TODO: a new iteration should consider only validation flags or only setting the data to NAN
bool xy_valid			# True if x and y are valid
bool z_valid			# True if z is valid
bool v_xy_valid		# True if vy and vy are valid
bool v_z_valid			# True if vz is valid
bool xy_global			# True if position (x, y) has a valid global reference (ref_lat, ref_lon)
bool z_global			# True if z has a valid global reference (ref_alt)
bool dist_bottom_valid		# True if distance to bottom surface is valid

# Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
uint64 ref_timestamp		# Time when reference position was set since system start, (microseconds)
float64 ref_lat		# Reference point latitude, (degrees). NaN if invalid/unknown
float64 ref_lon		# Reference point longitude, (degrees). NaN if invalid/unknown
float32 ref_alt		# Reference altitude AMSL, MUST be set to current (not at reference point!) ground level, (metres). NaN if invalid/unknown

# Estimator specified vehicle limits
float32 vxy_max		# Maximum horizontal speed - set to 0 when limiting not required (meters/sec)
float32 vz_max			# Maximum vertical speed - set to 0 when limiting not required (meters/sec)
float32 hagl_min	# Minimum height above ground level - set to 0 when limiting not required (meters)
float32 hagl_max	# Maximum height above ground level - set to 0 when limiting not required (meters)

# TOPICS vehicle_local_position vehicle_groundtruth vehicle_visual_odometry
