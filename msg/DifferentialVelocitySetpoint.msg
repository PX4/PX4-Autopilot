uint64 timestamp # time since system start (microseconds)

float32[2] velocity_ned # 2-dimensional velocity setpoint in NED frame
bool backwards	        # Flag for backwards driving
float32 yaw		# [rad] [-pi,pi] from North. Optional, NAN if not set. (Exclusive with velocity_ned, the rover will turn on the spot to reach the yaw setpoint)
