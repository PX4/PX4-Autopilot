# Main state, i.e. what user wants. Controlled by RC or from ground station via telemetry link.
uint8 MAIN_STATE_MANUAL = 0
uint8 MAIN_STATE_ALTCTL = 1
uint8 MAIN_STATE_POSCTL = 2
uint8 MAIN_STATE_AUTO_MISSION = 3
uint8 MAIN_STATE_AUTO_LOITER = 4
uint8 MAIN_STATE_AUTO_RTL = 5
uint8 MAIN_STATE_ACRO = 6
uint8 MAIN_STATE_OFFBOARD = 7
uint8 MAIN_STATE_STAB = 8
uint8 MAIN_STATE_RATTITUDE = 9
uint8 MAIN_STATE_AUTO_TAKEOFF = 10
uint8 MAIN_STATE_AUTO_LAND = 11
uint8 MAIN_STATE_AUTO_FOLLOW_TARGET = 12
uint8 MAIN_STATE_MAX = 13

uint64 timestamp # in microseconds since system start, is set whenever the writing thread stores new data

uint8 main_state		    	# main state machine
