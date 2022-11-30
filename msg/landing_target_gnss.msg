# Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

uint64 timestamp		# time since system start (microseconds)

int32 lat			    # Latitude in 1E-7 degrees
int32 lon			    # Longitude in 1E-7 degrees
int32 alt	            # Altitude above mean see level (AMSL) in 1E-3 meters , (millimetres)

float32 vx_rel			# X/north velocity  of target, relative to vehicle (navigation frame) [meters/second]
float32 vy_rel			# Y/east velocity of target, relative to vehicle (navigation frame) [meters/second]
float32 vz_rel			# Z/down velocity of target, relative to vehicle (navigation frame) [meters/second]

float32 cov_vx_rel		# X/north velocity variance [(meters/second)^2]
float32 cov_vy_rel		# Y/east velocity variance [(meters/second)^2]
float32 cov_vz_rel		# Z/down velocity variance [(meters/second)^2]
