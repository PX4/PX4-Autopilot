# Path setpoint in NED local frame
#
# Input to Guidance controller
# Needs to be kinematically consistent and feasible for smooth flight.
# setting a value to NaN means the state should not be controlled

uint32 MESSAGE_VERSION = 0

uint64 timestamp		# [us] Time since system start

float32[3] position		# [m] NED local frame position setpoint
float32[3] tangent		# Unit vector tangent to path in NED frame
float32 height_rate		# [m/s]
float32 curvature		# [1/m]
float32 equivalent_airspeed	# [m/s] NaN = not controlled
