# Trajectory setpoint in NED frame
# Input to position controller.

uint64 timestamp # time since system start (microseconds)

# NED local world frame
float32[3] position               # in meters
float32[3] velocity               # in meters/second
float32[3] acceleration           # in meters/second^2
float32[3] jerk                   # in meters/second^3 (for logging only)

float32[4] quaternion             # unit quaternion
float32[3] angular_velocity       # angular velocity in radians/second
