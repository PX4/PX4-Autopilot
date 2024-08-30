uint64 timestamp # time since system start (microseconds)

float32 forward_speed_setpoint 		  # [m/s] Desired forward speed for the rover
float32 forward_speed_setpoint_normalized # [-1, 1] Desired normalized forward speed for the rover
float32 steering_setpoint      		  # [rad/s] Desired steering for the rover
float32 steering_setpoint_normalized      # [-1, 1] Desired normalized steering for the rover

# TOPICS rover_ackermann_setpoint
