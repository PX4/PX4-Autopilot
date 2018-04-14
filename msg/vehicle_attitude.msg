# This is similar to the mavlink message ATTITUDE_QUATERNION_COV, but for onboard use

bool att_valid			# true if attitude is valid
bool att_rate_valid		# true if attitude rate is valid

float32 rollspeed		# Bias corrected angular velocity about X body axis in rad/s
float32 pitchspeed		# Bias corrected angular velocity about Y body axis in rad/s
float32 yawspeed		# Bias corrected angular velocity about Z body axis in rad/s

float32[4] q			# Quaternion rotation from NED earth frame to XYZ body frame
float32[4] delta_q_reset 	# Amount by which quaternion has changed during last reset
uint8 quat_reset_counter	# Quaternion reset counter

float32 att_std_dev		# Standard deviation of attitude error, (radians)
float32 att_rate_std_dev	# Standard deviation of attitude rate error, (radians/sec)

# TOPICS vehicle_attitude vehicle_attitude_groundtruth vehicle_vision_attitude
