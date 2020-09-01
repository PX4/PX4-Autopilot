uint64 timestamp		# time since system start (microseconds)

float32[24] states		# Internal filter states
uint8 n_states		# Number of states effectively used

float32[24] covariances	# Diagonal Elements of Covariance Matrix
