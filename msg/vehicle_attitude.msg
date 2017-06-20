# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use

float32 rollspeed	# Bias corrected angular velocity about X body axis in rad/s
float32 pitchspeed	# Bias corrected angular velocity about Y body axis in rad/s
float32 yawspeed	# Bias corrected angular velocity about Z body axis in rad/s

float32 roll_rate_bias		# Roll body angular rate bias (rad/s, x forward) - subtract from uncorrected gyro data
float32 pitch_rate_bias		# Pitch body angular rate bias (rad/s, y right) - subtract from uncorrected gyro data
float32 yaw_rate_bias		# Yaw body angular rate bias (rad/s, z down) - subtract from uncorrected gyro data

float32[4] q		# Quaternion rotation from NED earth frame to XYZ body frame
float32[4] delta_q_reset 	# Amount by which quaternion has changed during last reset
uint8 quat_reset_counter	# Quaternion reset counter

# TOPICS vehicle_attitude vehicle_attitude_groundtruth vehicle_vision_attitude
