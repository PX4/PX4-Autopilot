# Rover Position Setpoint

uint64 timestamp         # [us] Time since system start
float32[2] position_ned  # [m] [@range -inf, inf] [@frame NED] Target position
float32[2] start_ned     # [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to vehicle position] Start position which specifies a line for the rover to track
float32 cruising_speed   # [m/s] [@range 0, inf] [@invalid NaN Defaults to maximum speed] Cruising speed
float32 arrival_speed    # [m/s] [@range 0, inf] [@invalid NaN Defaults to 0] Speed the rover should arrive at the target with
float32 yaw              # [rad] [@range -pi,pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw] Mecanum only: Specify vehicle yaw during travel
