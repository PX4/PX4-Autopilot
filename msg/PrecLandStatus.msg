# Precision-landing runtime status: a single state captures both whether precision landing is active and which phase it is in.
#
# Published by: navigator (precland.cpp).
# Subscribed by: vision_target_estimator, flight_mode_manager (FlightTaskAuto).
#
# STOPPED is published when the precision-landing task is not active (just inactivated, or never started).
# The phase values (START..FALLBACK) are only published while the task is running and not yet finished.
# DONE is published once on successful completion, then STOPPED on the subsequent inactivation.

uint64 timestamp # [us] Time since system start

uint8 state # [@enum PREC_LAND_STATE] Current precision-landing state
uint8 PREC_LAND_STATE_STOPPED = 0 # Task not active (inactivated or never started)
uint8 PREC_LAND_STATE_START = 1 # Task just activated, initial setup
uint8 PREC_LAND_STATE_HORIZONTAL = 2 # Positioning over landing target while maintaining altitude
uint8 PREC_LAND_STATE_DESCEND = 3 # Descending while staying over the target
uint8 PREC_LAND_STATE_FINAL = 4 # Final landing approach (continues even without target in sight)
uint8 PREC_LAND_STATE_SEARCH = 5 # Searching for the landing target
uint8 PREC_LAND_STATE_FALLBACK = 6   # Fallback landing method (no precision)
uint8 PREC_LAND_STATE_DONE = 7 # Precision landing finished
