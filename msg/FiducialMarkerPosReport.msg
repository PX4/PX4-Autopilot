# Relative position of precision land target

uint64 timestamp		# [us] time since system start
uint64 timestamp_sample		# [us] timestamp of the raw observation

float32[3] rel_pos		# [m] Target position relative to vehicle, expressed in the frame defined by q
float32[3] cov_rel_pos		# [m^2] Target position variance, expressed in the frame defined by q

float32[4] q			# [-] Quaternion rotation from the rel_pos frame to the NED earth frame
