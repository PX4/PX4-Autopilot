# This is similar to the mavlink message ATTITUDE, but for onboard use */
uint64 timestamp	# in microseconds since system start
float32 rollspeed	# Angular velocity about body north axis (x) in rad/s
float32 pitchspeed	# Angular velocity about body east axis (y) in rad/s
float32 yawspeed	# Angular velocity about body down axis (z) in rad/s
float32[4] q		# Quaternion (NED)
bool q_valid		# Quaternion valid

# TOPICS vehicle_attitude vehicle_attitude_groundtruth
