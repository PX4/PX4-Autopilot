uint32 id			# ID of the estimator, commonly the component ID of the incoming message

uint64 timestamp_boot		# time of this estimate, in microseconds since system start
uint64 timestamp_computer	# timestamp provided by the companion computer, in us

float32 x			# X position in meters in NED earth-fixed frame
float32 y			# Y position in meters in NED earth-fixed frame
float32 z			# Z position in meters in NED earth-fixed frame (negative altitude)

float32 vx			# X velocity in meters per second in NED earth-fixed frame
float32 vy			# Y velocity in meters per second in NED earth-fixed frame
float32 vz			# Z velocity in meters per second in NED earth-fixed frame

float32[4] q			# Estimated attitude as quaternion
