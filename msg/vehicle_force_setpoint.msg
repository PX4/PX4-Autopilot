# Definition of force (NED) setpoint uORB topic. Typically this can be used
# by a position control app together with an attitude control app.


float32 x		# in N NED
float32 y		# in N NED
float32 z		# in N NED
float32 yaw		# right-hand rotation around downward axis (rad, equivalent to Tait-Bryan yaw)
