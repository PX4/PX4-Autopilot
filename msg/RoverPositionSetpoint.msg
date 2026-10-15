# Rover Position Setpoint

uint64 timestamp         # [us] Time since system start
float32[2] position_ned  # [m] [@range -inf, inf] [@frame NED] [@invalid NaN No target, the position controller stays idle] Target position
float32[2] start_ned     # [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to the vehicle position when the setpoint is received] Start position, the rover tracks the line from start to target
float32 cruising_speed   # [m/s] [@range -inf, inf] [@invalid NaN Defaults to maximum speed] Cruising speed, negative values drive in reverse with the rear pointing at the target
float32 arrival_speed    # [m/s] [@range 0, inf] [@invalid NaN Defaults to 0] Speed magnitude to arrive at the target with, independent of the direction of travel
float32 yaw              # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw] Only for vehicles that can move sideways (e.g. mecanum): vehicle yaw during travel
