uint64 timestamp # time since system start (microseconds)

float32 actual_speed # [m/s] Rover ground speed
float32 desired_speed # [m/s] Rover desired ground speed
float32 lookahead_distance # [m] Lookahead distance of pure the pursuit controller
float32 heading_error # [deg] Heading error of the pure pursuit controller
float32 pid_throttle_integral # [-1, 1] Integral of the PID for the normalized throttle to control the rover speed during missions
float32 crosstrack_error # [m] Shortest distance from the vehicle to the path

# TOPICS rover_ackermann_guidance_status
