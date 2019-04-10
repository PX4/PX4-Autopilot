# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

uint64 timestamp	# time since system start (microseconds)

float32 yawspeed	# in radians/sec
float32 speed_xy	# in meters/sec
float32 speed_up	# in meters/sec
float32 speed_down	# in meters/sec
float32 tilt    # in radians [0, PI]
float32 min_distance_to_ground # in meters
float32 max_distance_to_ground # in meters
