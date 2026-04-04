# Diagnostic record of a Navigator mode transition, published once per change.
# Internal / ULog only; not streamed over MAVLink.

uint64 timestamp                # time since system start (microseconds)

uint8 nav_state_prev            # previous nav_state (VehicleStatus NAVIGATION_STATE_*), NAV_STATE_NONE if none
uint8 nav_state_new             # new nav_state (VehicleStatus NAVIGATION_STATE_*), NAV_STATE_NONE if none
uint8 nav_state_user_intention  # user-intended nav_state at time of transition (from vehicle_status)

bool in_failsafe                # vehicle_status.failsafe at time of transition
bool triplet_reset_applied      # reset_triplets() was called on entry

float32 entry_velocity_xy       # horizontal ground speed at transition [m/s]
float32 entry_dist_to_target    # horizontal distance from current position to new current setpoint [m], NAN if no valid target

uint8 NAV_STATE_NONE = 255      # sentinel for nav_state_prev / nav_state_new when no navigator mode is active
