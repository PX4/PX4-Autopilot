uint64 timestamp	# time since system start (microseconds)

float32 roll	    # body angular rates in NED frame
float32 pitch	    # body angular rates in NED frame
float32 yaw			# body angular rates in NED frame

# For clarification: For multicopters thrust_x and thrust_y are usually 0 and thrust_z is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32 thrust_x	   # Throttle command in body x direction [-1,1]
float32 thrust_y	   # Throttle command in body y direction [-1,1]
float32 thrust_z	   # Throttle command in body z direction [-1,1]
