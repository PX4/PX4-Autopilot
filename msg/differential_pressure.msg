uint64 timestamp				# Microseconds since system boot, needed to integrate
uint64 error_count				# Number of errors detected by driver
float32 differential_pressure_raw_pa		# Raw differential pressure reading (may be negative)
float32 differential_pressure_filtered_pa	# Low pass filtered differential pressure reading
float32 max_differential_pressure_pa		# Maximum differential pressure reading
float32 temperature				# Temperature provided by sensor, -1000.0f if unknown
