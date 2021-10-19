uint64 timestamp # time since system start (microseconds)

uint8 mode # what mode is requested according to commander_state.MAIN_STATE_

uint8 source # how the request was triggered
uint8 SOURCE_RC_MODE_SLOT = 0
uint8 SOURCE_RC_SWITCH = 1
