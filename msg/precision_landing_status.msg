# ORBIT_YAW_BEHAVIOUR
uint8 PRECLAND_STATE_START = 0
uint8 PRECLAND_STATE_HORIZONTAL_APPROACH = 1
uint8 PRECLAND_STATE_DESCEND_ABOVE_TARGET = 2
uint8 PRECLAND_STATE_FINAL_APPROACH = 3
uint8 PRECLAND_STATE_SEARCH = 4
uint8 PRECLAND_STATE_FALLBACK = 5

uint64 timestamp 	# time since system start (microseconds)
uint8 precland_state    # Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise. [m]
