# Mode completion result, published by an active mode.
# Note that this is not always published (e.g. when a user switches modes or on
# failsafe activation)
uint64 timestamp				 # time since system start (microseconds)


uint8 RESULT_SUCCESS = 0
# [1-99]: reserved
uint8 RESULT_FAILURE_OTHER = 100 # Mode failed (generic error)

uint8 result                     # One of RESULT_*

uint8 nav_state                  # Source mode

