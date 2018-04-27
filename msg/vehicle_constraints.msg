# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

float32 yawspeed	# in radians/sec
float32 speed_xy	# in meters/sec
float32 speed_up	# in meters/sec
float32 speed_down	# in meters/sec
float32 tilt    # in radians [0, PI]

bool landing_gear_up # true when up, false when down
