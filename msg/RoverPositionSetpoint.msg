# Rover Position Setpoint

uint64 timestamp # [us] Time since system start
float32[2] target_ned # [m] [@range -inf, inf] [@frame NED] [@invalid NaN No target, the position controller stays idle unless a course is set] Target position, ignored while a course is set
float32[2] start_ned # [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to the vehicle position when the setpoint is received] Start position of the line the rover tracks, towards target_ned or along course
float32 cruising_speed # [m/s] [@range -inf, inf] [@invalid NaN Defaults to maximum speed] Cruising speed, negative values drive in reverse with the rear pointing at the target
float32 arrival_speed # [m/s] [@range 0, inf] [@invalid NaN Defaults to 0] Speed magnitude to arrive at the target with, independent of the direction of travel
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw] Only for vehicles that can move sideways (e.g. mecanum): vehicle yaw during travel
float32 course # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN No course, track target_ned instead] Course over ground to hold along the line through start_ned, takes precedence over target_ned. 0 is a valid course (north), publishers must set NaN when not holding a course
