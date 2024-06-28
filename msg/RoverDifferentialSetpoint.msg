uint64 timestamp # time since system start (microseconds)

float32 speed # [m/s] collective roll-off speed in body x-axis
bool closed_loop_speed_control # true if speed is controlled using estimator feedback, false if direct feed-forward
float32 yaw_rate # [rad/s] yaw rate
bool closed_loop_yaw_rate_control # true if yaw rate is controlled using gyroscope feedback, false if direct feed-forward

# TOPICS rover_differential_setpoint rover_differential_control_output
