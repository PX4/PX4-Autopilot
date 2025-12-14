# Relative orientation of landing target in NED (North, East, Down)

uint64 timestamp		# time since system start (microseconds)

uint64 timestamp_sample		# the timestamp of the observation (microseconds)

float32 yaw_ned      # orientation of the target relative to the NED frame [rad] [-Pi ; Pi]
float32 yaw_var_ned  # orientation uncertainty [rad^2]
