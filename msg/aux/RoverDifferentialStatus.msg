uint64 timestamp # time since system start (microseconds)

float32 actual_speed	         # [m/s] Actual forward speed of the rover
float32 actual_yaw               # [rad] Actual yaw of the rover
float32 actual_yaw_rate          # [rad/s] Actual yaw rate of the rover
float32 desired_yaw_rate         # [rad/s] Yaw rate setpoint for the closed loop yaw rate controller
float32 forward_speed_normalized # [-1, 1] Normalized forward speed setpoint
float32 speed_diff_normalized    # [-1, 1] Normalized speed difference setpoint between the left and right motor
float32 pid_yaw_integral         # Integral of the PID for the closed loop yaw controller
float32 pid_yaw_rate_integral    # Integral of the PID for the closed loop yaw rate controller
float32 pid_throttle_integral    # Integral of the PID for the closed loop speed controller

# TOPICS rover_differential_status
