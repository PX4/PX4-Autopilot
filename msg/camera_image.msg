uint64 timestamp_utc		# Capture time in UTC / GPS time
float64 lat					# Latitude in degrees (WGS84)
float64 lon					# Longitude in degrees (WGS84)
float32 alt					# Altitude (AMSL)
float32 relative_alt			# Altitude relative to home
float32[4] q					# Attitude of the camera, pointing down is XXX
uint32 seq					# Image sequence number
bool result					# True for success, false for failure
