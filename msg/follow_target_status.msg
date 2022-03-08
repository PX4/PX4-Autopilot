uint64 timestamp             # time since system start (microseconds)

float32[3] pos_est_filtered  # Filtered target position used for vehicle's setpoint generation
float32[3] vel_est_filtered  # Filtered Target velocity used for vehicle's setpoint generation

float32 tracked_target_orientation # [rad] Tracked target orientation, which can be rate constrained
float32 follow_angle               # [rad] Current follow angle setting ,which can be rate constrained
float32 current_orbit_angle        # [rad] Current orbit angle value, which is the final command output

float32 drone_to_target_heading
float32[2] orbit_tangential_velocity

bool emergency_ascent
float32 gimbal_pitch
