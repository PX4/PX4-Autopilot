uint64 timestamp # time since system start (microseconds)

bool valid			# Flag indicating whether the time estiamtes are valid
float32 time_estimate		# [s] Estimated time for RTL
float32 safe_time_estimate	# [s] Same as time_estimate, but with safety factor and safety margin included (factor*t + margin)
