uint64 timestamp # [us] Time since system start

uint8 STATE_INACTIVE = 0 # payload board is inactive
uint8 STATE_CHARGING = 1 # payload board is charging
uint8 STATE_READY = 2 # payload board is ready
uint8 STATE_ACTIVE = 3 # payload board is active
uint8 STATE_ERR = 100 # payload board is in an error state
uint8 state # [@enum STATE] state reported by the payload board

uint8 countdown # [s] remaining time reported by the payload board
uint8[16] custom_message # [-] payload board specific debug information
