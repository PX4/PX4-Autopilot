uint32 id			# ID of the estimator, commonly the component ID of the incoming message

uint64 timestamp_received	# timestamp when the estimate was received

float32 x			# X velocity in meters per second in NED earth-fixed frame
float32 y			# Y velocity in meters per second in NED earth-fixed frame
float32 z			# Z velocity in meters per second in NED earth-fixed frame

float32 vel_err	    # velocity error 1-std for each axis (metres)
