# This is similar to the mavlink message CONTROL_SYSTEM_STATE, but for onboard use */
uint64 timestamp		# in microseconds since system start
float32 x_acc			# X acceleration in body frame
float32 y_acc			# Y acceleration in body frame
float32 z_acc			# Z acceleration in body frame
float32 x_vel			# X velocity in body frame
float32 y_vel			# Y velocity in body frame
float32 z_vel			# Z velocity in body frame
float32 x_pos			# X position in local frame
float32 y_pos			# Y position in local frame
float32 z_pos			# z position in local frame
float32 airspeed		# Airspeed, estimated
bool airspeed_valid		# False: Non-finite values or non-updating sensor
float32[3] vel_variance	# Variance in body velocity estimate
float32[3] pos_variance	# Variance in local position estimate
float32[4] q 			# Attitude Quaternion
float32 roll_rate		# Roll body angular rate (rad/s, x forward/y right/z down)
float32 pitch_rate		# Pitch body angular rate (rad/s, x forward/y right/z down)
float32 yaw_rate		# Yaw body angular rate (rad/s, x forward/y right/z down)
float32 horz_acc_mag		# Magnitude of the horizontal acceleration
