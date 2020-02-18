# Bezier Trajectory description. See also Mavlink TRAJECTORY msg
# The topic trajectory_bezier describe each waypoint defined in vehicle_trajectory_bezier

uint64 timestamp		# time since system start (microseconds)

float32[3] position
float32 yaw
float32 delta
