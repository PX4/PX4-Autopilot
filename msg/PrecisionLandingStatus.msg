uint8 PRECLAND_STATE_AUTORTL = 0               # Execute normal RTL logic until the target is spotted
uint8 PRECLAND_STATE_SEARCH = 1                # Drone is supposed to land at current location, but has not spotted the target yet. Wait at current location in hopes of spotting the target.
uint8 PRECLAND_STATE_MOVE_ABOVE_TARGET = 2     # Fly above target while maintaining RTL altitude
uint8 PRECLAND_STATE_DESCEND_ABOVE_TARGET = 3  # Drone is above target and descending. This state still expects to see the target.
uint8 PRECLAND_STATE_TOUCHING_DOWN = 4         # Same as descend, but if sight of target is lost, drone continues to land at last known location.
uint8 PRECLAND_STATE_FALLBACK = 5              # Fallback to conventional landing, either because of opportunistic precision landing or because maximum number of precland attempts were reached

uint64 timestamp        # time since system start (microseconds)
uint8 state    # this flight-task's internal state
