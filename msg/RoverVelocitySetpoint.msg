uint64 timestamp # time since system start (microseconds)

float32 speed   # [m/s] [-inf, inf] Speed setpoint (Backwards driving if negative)
float32 bearing # [rad] [-pi,pi] from North.
float32 yaw 	# [rad] [-pi, pi] (Mecanum only, Optional, defaults to current vehicle yaw) Vehicle yaw setpoint in NED frame
