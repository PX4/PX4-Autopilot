# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use
float32 rollspeed	# Angular velocity about body north axis (x) in rad/s
float32 pitchspeed	# Angular velocity about body east axis (y) in rad/s
float32 yawspeed	# Angular velocity about body down axis (z) in rad/s
float32[4] q		# Quaternion (NED)

# TOPICS vehicle_attitude vehicle_attitude_groundtruth vehicle_vision_attitude
