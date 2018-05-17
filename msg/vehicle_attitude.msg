# This is similar to the mavlink message ATTITUDE_QUATERNION_COV, but for onboard use

float32 rollspeed		# Bias corrected angular velocity about X body axis in rad/s. NaN if invalid/unknown
float32 pitchspeed		# Bias corrected angular velocity about Y body axis in rad/s. NaN if invalid/unknown
float32 yawspeed		# Bias corrected angular velocity about Z body axis in rad/s. NaN if invalid/unknown

float32[4] q			# Quaternion rotation from NED earth frame to XYZ body frame. NaN if invalid/unknown
float32[4] delta_q_reset 	# Amount by which quaternion has changed during last reset
uint8 quat_reset_counter	# Quaternion reset counter

float32[9] covariance		# Row-major representation of attitude covariance. If invalid/unknown, first cell is NaN

float32 att_std_dev		# Standard deviation of attitude error, (radians). NaN if unknown
float32 att_rate_std_dev	# Standard deviation of attitude rate error, (radians/sec). NaN if unknown

# TOPICS vehicle_attitude vehicle_attitude_groundtruth vehicle_vision_attitude
