uint64 timestamp		# Timestamp in microseconds since boot
float32[32] states		# Internal filter states
float32 n_states		# Number of states effectively used
uint8 nan_flags			# Bitmask to indicate NaN states
uint8 health_flags		# Bitmask to indicate sensor health states (vel, pos, hgt)
uint8 timeout_flags		# Bitmask to indicate timeout flags (vel, pos, hgt)
float32[28] covariances	# Diagonal Elements of Covariance Matrix
