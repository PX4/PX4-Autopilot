# Precision takeoff status.
#
# Published by: navigator (prec_takeoff.cpp).
# Subscribed by: vision_target_estimator, flight_mode_manager (FlightTaskAuto).
#
# ONGOING while a vertical takeoff setpoint is active and MIS_TKO_PREC is set. DONE is published once when the
# takeoff altitude is reached, then STOPPED once the takeoff is no longer active. Only transitions are published.

uint64 timestamp # [us] Time since system start

uint8 state # [@enum PREC_TAKEOFF_STATE] Current precision-takeoff state
uint8 PREC_TAKEOFF_STATE_STOPPED = 0 # Not active
uint8 PREC_TAKEOFF_STATE_ONGOING = 1 # Takeoff active, the current setpoint tracks the landing target
uint8 PREC_TAKEOFF_STATE_DONE = 2 # Takeoff altitude reached
