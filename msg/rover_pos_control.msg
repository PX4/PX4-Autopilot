uint64 timestamp		# time since system start (microseconds)

float32[3] velocity_body	# [m/s] Velocity in body frame
float32 raw_yaw_torque_setpoint	#[rad/s^2] Yaw torque setpoint directly out of the rate PID controller
float32 yaw_control_scalar 	# Scalar applied for Yaw actuator control input, which is inverse to the square of forward speed
float32 raw_steering_input 	# [rad/s^2] Raw steering input which is raw torque setpoint * yaw control scalar
