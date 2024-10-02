uint64 timestamp # time since system start (microseconds)

float32 forward_speed_setpoint            # [m/s] Desired forward speed
float32 forward_speed_setpoint_normalized # [-1, 1] Desired normalized forward speed
float32 lateral_speed_setpoint            # [m/s] Desired lateral speed
float32 lateral_speed_setpoint_normalized # [-1, 1] Desired normalized lateral speed
float32 yaw_rate_setpoint                 # [rad/s] Desired yaw rate
float32 yaw_rate_setpoint_normalized      # [-1, 1] Desired normalized yaw rate
float32 yaw_setpoint 	                  # [rad] Desired yaw (heading)

# TOPICS rover_mecanum_setpoint
