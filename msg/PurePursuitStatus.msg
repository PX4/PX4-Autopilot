# Pure pursuit status

uint64 timestamp              # [us] Time since system start
float32 lookahead_distance    # [m] [@range 0, inf] Lookahead distance of pure the pursuit controller
float32 target_bearing        # [rad] [@range -pi, pi] [@frame NED] Target bearing calculated by the pure pursuit controller
float32 crosstrack_error      # [m] [@range -inf (Left of the path), inf (Right of the path)] Shortest distance from the vehicle to the path
float32 distance_to_waypoint  # [m] [@range -inf, inf]Distance from the vehicle to the current waypoint
float32 bearing_to_waypoint   # [rad] [@range -pi, pi] [@frame NED]Bearing towards current waypoint
