# VEHICLE_VTOL_STATE, should match 1:1 MAVLinks's MAV_VTOL_STATE
uint8 VEHICLE_VTOL_STATE_UNDEFINED = 0
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_FW = 1
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_MC = 2
uint8 VEHICLE_VTOL_STATE_MC = 3
uint8 VEHICLE_VTOL_STATE_FW = 4

uint64 timestamp			# time since system start (microseconds)

uint8 vehicle_vtol_state		# current state of the vtol, see VEHICLE_VTOL_STATE

bool fixed_wing_system_failure		# vehicle in fixed-wing system failure failsafe mode (after quad-chute)
