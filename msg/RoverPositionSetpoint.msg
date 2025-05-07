# Rover Position Setpoint

uint64     timestamp      # Time since system start [us]
float32[2] position_ned   # Target position [m] [@range -inf, inf] [@frame NED]
float32[2] start_ned	  # Start position which specifies a line for the rover to track [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to vehicle position]
float32    cruising_speed # Cruising speed [m/s] [@range 0, inf] [@invalid NaN Defaults to maximum speed]
float32    arrival_speed  # Speed the rover should arrive at the target with [m/s] [@range 0, inf] [@invalid NaN Defaults to 0]
float32    yaw            # Mecanum only: Specify vehicle yaw during travel [rad] [@range -pi,pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw]
