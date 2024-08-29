uint64 timestamp # time since system start (microseconds)

float32 actual_speed	       # [m/s] Actual forward speed of the rover
float32 desired_yaw_rate_deg_s # [deg/s] Desired yaw rate
float32 actual_yaw_rate_deg_s  # [deg/s] Actual yaw rate of the rover
float32 pid_yaw_rate_integral  # Integral of the PID for the desired yaw rate controller

# TOPICS rover_differential_status
