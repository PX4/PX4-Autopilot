uint64 timestamp				# time since system start (microseconds)
uint64 error_count				# Number of errors detected by driver
float32 differential_pressure_raw_pa		# Raw differential pressure reading (may be negative)
float32 differential_pressure_filtered_pa	# Low pass filtered differential pressure reading
float32 temperature				# Temperature provided by sensor, -1000.0f if unknown
uint32 device_id				# unique device ID for the sensor that does not change between power cycles
