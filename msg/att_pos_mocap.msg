uint32 id			# ID of the estimator, commonly the component ID of the incoming message

uint64 timestamp_received	# timestamp when the estimate was received

float32[4] q			# Estimated attitude as quaternion

float32 x			# X position in meters in NED earth-fixed frame
float32 y			# Y position in meters in NED earth-fixed frame
float32 z			# Z position in meters in NED earth-fixed frame (negative altitude)
