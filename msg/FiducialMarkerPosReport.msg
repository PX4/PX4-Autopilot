# Relative position of precision land target

uint64 timestamp		# time since system start (microseconds)

uint64 timestamp_sample		# the timestamp of the observation (microseconds)

float32[3] rel_pos		# [x,y,z] Target position relative to vehicle [m], expressed in the frame defined by q
float32[3] cov_rel_pos		# [x,y,z] Position variance [m^2]

float32[4] q			# Quaternion rotation from the rel_pos frame to the NED earth frame
