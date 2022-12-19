uint64 timestamp # [us] time since system start
float32 lateral_touchdown_offset # [m] lateral touchdown position offset manually commanded during landing
bool flaring # true if the aircraft is flaring

# abort status is:
# 0 if not aborted
# >0 if aborted, with the singular abort criterion which triggered the landing abort enumerated by the following abort reasons
uint8 abort_status

# abort reasons
# after the manual operator abort, corresponds to individual bits of param FW_LND_ABORT
uint8 NOT_ABORTED = 0
uint8 ABORTED_BY_OPERATOR = 1
uint8 TERRAIN_NOT_FOUND = 2 # FW_LND_ABORT (1 << 0)
uint8 TERRAIN_TIMEOUT = 3 # FW_LND_ABORT (1 << 1)
uint8 UNKNOWN_ABORT_CRITERION = 4
