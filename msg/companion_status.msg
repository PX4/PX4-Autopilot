uint64 timestamp			# time since system start (microseconds)
uint8 state				    # state
uint8 type				    # type of source on the companion emitting the state
uint16 pid				    # process ID of source on the companion emitting the state

uint8 COMPANION_STATE_HEALTHY = 0
uint8 COMPANION_STATE_TIMEOUT = 1
uint8 COMPANION_STATE_ABORT = 2
uint8 COMPANION_STATE_STARTING = 3
uint8 COMPANION_STATE_COMPONENT_FAIL = 4

uint8 COMPANION_TYPE_GENERIC = 0
uint8 COMPANION_TYPE_AVOIDANCE = 1
uint8 COMPANION_TYPE_VIO = 2

