uint64 timestamp	# time since system start (microseconds)

# body angular rates in NED frame
float32 roll		# [rad/s] roll rate setpoint
float32 pitch		# [rad/s] pitch rate setpoint
float32 yaw		# [rad/s] yaw rate setpoint

# [rad/s] yaw rate setpoint the wheel attempts to track (the wheel controller has different gains than the yaw controller)
float32 yaw_via_wheel

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] thrust_body	# Normalized thrust command in body NED frame [-1,1]
