# Pure pursuit status

uint64  timestamp            # Time since system start [us]
float32 lookahead_distance   # Lookahead distance of pure the pursuit controller [m] [@range 0, inf]
float32 target_bearing       # Target bearing calculated by the pure pursuit controller [rad] [@range -pi, pi] [@frame NED]
float32 crosstrack_error     # Shortest distance from the vehicle to the path [m] [@range -inf (Left of the path), inf (Right of the path)]
float32 distance_to_waypoint # Distance from the vehicle to the current waypoint [m] [@range -inf, inf]
float32 bearing_to_waypoint  # Bearing towards current waypoint [rad] [@range -pi, pi] [@frame NED]
