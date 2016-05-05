# This is similar to the mavlink message ATTITUDE, but for onboard use */
uint64 timestamp	# in microseconds since system start
# @warning roll, pitch and yaw have always to be valid, the rotation matrix and quaternion are optional
float32 roll		# Roll angle (rad, Tait-Bryan, NED)
float32 pitch		# Pitch angle (rad, Tait-Bryan, NED)
float32 yaw		# Yaw angle (rad, Tait-Bryan, NED)
float32 rollspeed	# Roll body angular rate (rad/s, x forward/y right/z down)
float32 pitchspeed	# Pitch body angular rate (rad/s, x forward/y right/z down)
float32 yawspeed	# Yaw body angular rate (rad/s, x forward/y right/z down)
float32 rollacc		# Roll angular accelration (rad/s^2, x forward/y right/z down)
float32 pitchacc	# Pitch angular acceleration (rad/s^2, x forward/y right/z down)
float32 yawacc		# Yaw angular acceleration (rad/s^2, x forward/y right/z down)
float32 rate_vibration	# Value between 0 and 1 indicating vibration. A value of 0 means no vibration, a value of 1 indicates unbearable vibration levels.
float32 accel_vibration	# Value between 0 and 1 indicating vibration. A value of 0 means no vibration, a value of 1 indicates unbearable vibration levels.
float32 mag_vibration	# Value between 0 and 1 indicating vibration. A value of 0 means no vibration, a value of 1 indicates unbearable vibration levels.
float32[3] rate_offsets	# Offsets of the body angular rates from zero
float32[9] R		# Rotation matrix, body to world, (Tait-Bryan, NED)
float32[4] q		# Quaternion (NED)
float32[3] g_comp	# Compensated gravity vector
bool R_valid		# Rotation matrix valid
bool q_valid		# Quaternion valid
