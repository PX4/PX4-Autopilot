# Status of the launch detection state machine (fixed-wing only)

uint64 timestamp # time since system start (microseconds)

uint8 STATE_WAITING_FOR_LAUNCH 			= 0 # waiting for launch
# value 1 was STATE_LAUNCH_DETECTED_DISABLED_MOTOR, the motor lock moved to the control allocator (CA_R_LK_DELAY)
uint8 STATE_FLYING 				= 2 # launch detected, use normal takeoff/flying configuration

uint8 launch_detection_state
