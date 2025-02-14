uint64 timestamp # time since system start (microseconds)

float32 forward_speed_setpoint 		  # [m/s] Desired forward speed for the rover
float32 forward_speed_setpoint_normalized # [-1, 1] Normalized forward speed for the rover
float32 yaw_rate_setpoint      		  # [rad/s] Desired yaw rate for the rover (Overriden by yaw controller if yaw_setpoint is used)
float32 speed_diff_setpoint_normalized    # [-1, 1] Normalized speed difference between the left and right wheels
float32 yaw_setpoint 	       		  # [rad] Desired yaw (heading) for the rover

# TOPICS rover_differential_setpoint
