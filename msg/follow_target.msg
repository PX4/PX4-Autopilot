uint64 timestamp		# timestamp, UNIX epoch (GPS synced)
uint8 est_capabilities		# Flags to indicate which quantities the tracker knows about: (POS, VEL, ACCEL, ATT + RATES)
uint32 lat			# target position (deg * 1e7)
uint32 lon			# target position (deg * 1e7) 
float32 alt			# target position 
float32[3] velocity		# target velocity (AMSL, meters) # target position (0 0 0 for unknown)
float32[3] accel		# target acceleration (linear) in earth frame. 
float32[4] attitude_q		# where the target is facing. 
float32[3] rates		# (0 0 0 for unknown)
float32[3] pos_cov		# uncertainly in earth frame for X, Y and Z. We will need to agree on the exact format here -1 for unknown
uint64 custom_state		# A custom vector, can be used to transmit e.g. button states or switches of a tracker device
