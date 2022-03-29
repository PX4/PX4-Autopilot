uint64 timestamp             # [microseconds] time since system start

float32[3] pos_est_filtered  # [m/s] Filtered target position used for vehicle's setpoint generation
float32[3] vel_est_filtered  # [m/s] Filtered Target velocity used for vehicle's setpoint generation

float32 tracked_target_course # [rad] Tracked target course in NED local frame (North is course zero)
float32 follow_angle          # [rad] Current follow angle setting
float32 orbit_angle_setpoint  # [rad] Current orbit angle setpoint from the smooth trajectory generator

bool emergency_ascent
float32 gimbal_pitch
