# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

uint64 timestamp	# time since system start (microseconds)

#value of 0 means no constraint along this axis, value of 1 means no movements alowed, value bigger than 1 forces negative movement.
float32[2] constraints_normalized_x   # constraints determined by range sensor measurements [x negative, x positive]
float32[2] constraints_normalized_y   # constraintss determined by range sensor measurements [y negative, y positive]

float32[2] original_setpoint   # velocities demanded
float32[2] adapted_setpoint    # velocities allowed

