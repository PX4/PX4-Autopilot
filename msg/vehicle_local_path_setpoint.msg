# Local position setpoint in NED frame
# setting something to NaN means the state should not be controlled

uint64 timestamp	# time since system start (microseconds)

float32[2] prev_wp	# previous waypoint in meters NED
float32 x		# in meters NED
float32 y		# in meters NED
float32 z		# in meters NED
float32 vx		# in meters/sec
float32 vy		# in meters/sec
float32 vz		# in meters/sec
float32 true_airspeed	# true air relative setpoint in meters/sec
float32 curvature	# in meters/sec^2

# TOPICS vehicle_local_path_setpoint
