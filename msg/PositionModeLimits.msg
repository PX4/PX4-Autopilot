# Teomporary limits to allow slow down in position mode, need to be streamed with 1Hz otherwise the defaults apply again
uint64 timestamp # time since system start (microseconds)

float32 horizontal_velocity_limit # [m/s] movement parallel to local frame ground plane
float32 vertical_velocity_limit # [m/s] movement perpendicular to local frame ground plane
float32 yaw_rate_limit # [rad/s] turn rate around vehicle yaw axis
