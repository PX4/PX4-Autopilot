# Relative orientation of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames


uint64 timestamp		# time since system start (microseconds)

uint64 timestamp_sample		# the timestamp of the raw data (microseconds)

float32[3] acc_xyz			# Downsampled UAV bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2

float32[4] q_att            	# Downsampled UAV quaternion rotation from the FRD body frame to the NED earth frame
