# Path setpoint in NED frame
#
# Input to Guidance controller
# Needs to be kinematically consistent and feasible for smooth flight.
# setting a value to NaN means the state should not be controlled

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start

float64 lat # [deg] Latitude
float64 lon # [deg] Longitude
float32 alt # [m] Altitude AMSL
# NED local world frame
float32[3] tangent # Unit vector tangent to path
float32 height_rate
float32 curvature
