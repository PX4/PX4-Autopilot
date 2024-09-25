uint64 timestamp # time since system start (microseconds)

float32 measured_forward_speed        # [m/s] Measured speed in body x direction. Positiv: forwards, Negativ: backwards
float32 measured_lateral_speed        # [m/s] Measured speed in body y direction. Positiv: right, Negativ: left
float32 adjusted_yaw_rate_setpoint    # [rad/s] Yaw rate setpoint output of the closed loop yaw controller
float32 measured_yaw_rate  	      # [rad/s] Measured yaw rate
float32 measured_yaw        	      # [rad] Measured yaw
float32 pid_yaw_rate_integral  	      # Integral of the PID for the closed loop yaw rate controller
float32 pid_yaw_integral       	      # Integral of the PID for the closed loop yaw controller
float32 pid_forward_throttle_integral # Integral of the PID for the closed loop forward speed controller
float32 pid_lateral_throttle_integral # Integral of the PID for the closed loop lateral speed controller

# TOPICS rover_mecanum_status
