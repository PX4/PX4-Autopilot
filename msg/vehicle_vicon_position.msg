uint64 timestamp		# time of this estimate, in microseconds since system start
bool valid			# true if position satisfies validity criteria of estimator

float32 x			# X position in meters in NED earth-fixed frame
float32 y			# Y position in meters in NED earth-fixed frame
float32 z			# Z position in meters in NED earth-fixed frame (negative altitude)
float32 roll
float32 pitch
float32 yaw
float32[4] q			# Attitude as quaternion
