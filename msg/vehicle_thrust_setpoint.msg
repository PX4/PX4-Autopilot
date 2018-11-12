
uint64 timestamp		# time since system start (microseconds)

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] thrust_body					# Normalized thrust command in body NED frame [-1,1]

# TOPICS vehicle_thrust_setpoint mc_virtual_thrust_setpoint fw_virtual_thrust_setpoint
