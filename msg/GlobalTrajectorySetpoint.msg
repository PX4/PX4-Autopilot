# Trajectory setpoint in NED frame
# Input to PID position controller.
# Needs to be kinematically consistent and feasible for smooth flight.
# setting a value to NaN means the state should not be controlled

uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt			# Altitude AMSL, (meters)
# NED local world frame
float32[3] velocity # in meters/second
float32[3] acceleration # in meters/second^2
float32[3] jerk # in meters/second^3 (for logging only)
