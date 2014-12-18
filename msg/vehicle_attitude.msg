# This is similar to the mavlink message ATTITUDE, but for onboard use */
uint64 timestamp	# in microseconds since system start
# @warning roll, pitch and yaw have always to be valid, the rotation matrix and quaternion are optional
float32 roll		# Roll angle (rad, Tait-Bryan, NED)
float32 pitch		# Pitch angle (rad, Tait-Bryan, NED)
float32 yaw		# Yaw angle (rad, Tait-Bryan, NED)
float32 rollspeed	# Roll angular speed (rad/s, Tait-Bryan, NED)
float32 pitchspeed	# Pitch angular speed (rad/s, Tait-Bryan, NED)
float32 yawspeed	# Yaw angular speed (rad/s, Tait-Bryan, NED)
float32 rollacc		# Roll angular accelration (rad/s, Tait-Bryan, NED)
float32 pitchacc	# Pitch angular acceleration (rad/s, Tait-Bryan, NED)
float32 yawacc		# Yaw angular acceleration (rad/s, Tait-Bryan, NED)
float32[3] rate_offsets	# Offsets of the body angular rates from zero
float32[9] R		# Rotation matrix, body to world, (Tait-Bryan, NED)
float32[4] q		# Quaternion (NED)
float32[3] g_comp		# Compensated gravity vector
bool R_valid		# Rotation matrix valid
bool q_valid		# Quaternion valid

# secondary attitude for VTOL
float32 roll_sec		# Roll angle (rad, Tait-Bryan, NED)
float32 pitch_sec		# Pitch angle (rad, Tait-Bryan, NED)
float32 yaw_sec			# Yaw angle (rad, Tait-Bryan, NED)
float32 rollspeed_sec		# Roll angular speed (rad/s, Tait-Bryan, NED)
float32 pitchspeed_sec		# Pitch angular speed (rad/s, Tait-Bryan, NED)
float32 yawspeed_sec	    	# Yaw angular speed (rad/s, Tait-Bryan, NED)
float32 rollacc_sec		# Roll angular accelration (rad/s, Tait-Bryan, NED)
float32 pitchacc_sec	    	# Pitch angular acceleration (rad/s, Tait-Bryan, NED)
float32 yawacc_sec		# Yaw angular acceleration (rad/s, Tait-Bryan, NED)
float32[3] rate_offsets_sec	# Offsets of the body angular rates from zero
float32[9] R_sec		# Rotation matrix, body to world, (Tait-Bryan, NED)
float32[4] q_sec		# Quaternion (NED)
float32[3] g_comp_sec		# Compensated gravity vector
bool R_valid_sec		# Rotation matrix valid
bool q_valid_sec		# Quaternion valid
