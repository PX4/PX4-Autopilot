uint64 timestamp			# time since system start (microseconds)
uint8 state				    # state
uint8 source				# source on the companion emitting the state

uint8 COMPANION_STATE_HEALTHY = 0
uint8 COMPANION_STATE_TIMEOUT = 1
uint8 COMPANION_STATE_ABORT = 2
uint8 COMPANION_STATE_STARTING = 3
uint8 COMPANION_STATE_COMPONENT_FAIL = 4

uint8 COMPANION_SOURCE_GENERIC = 0
uint8 COMPANION_SOURCE_AVOIDANCE = 1
uint8 COMPANION_SOURCE_VIO = 2

