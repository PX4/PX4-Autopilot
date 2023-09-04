# PREC_LAND_STATUS message data
uint8 PREC_LAND_STATE_STOPPED = 0
uint8 PREC_LAND_STATE_ONGOING = 1

uint8 PREC_LAND_NAV_STATE_START = 0
uint8 PREC_LAND_NAV_STATE_HORIZONTAL = 1
uint8 PREC_LAND_NAV_STATE_DESCEND = 2
uint8 PREC_LAND_NAV_STATE_FINAL = 3
uint8 PREC_LAND_NAV_STATE_SEARCH = 4
uint8 PREC_LAND_NAV_STATE_FALLBACK = 5
uint8 PREC_LAND_NAV_STATE_DONE = 6

uint64 timestamp			# time since system start (microseconds)

uint8 state    # PX4 state of delivery (PREC_LAND_STATE_*)
uint8 nav_state		# Precision delivery state (PREC_LAND_NAV_STATE_*)
