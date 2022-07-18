uint64 timestamp # [us] time since system start
float32 lateral_touchdown_offset # [m] lateral touchdown position offset manually commanded during landing
bool flaring # true if the aircraft is flaring

# abort status is:
# 0 if not aborted
# >0 if aborted, with the singular abort criterion which triggered the landing abort enumerated by the following abort reasons
uint8 abort_status

# abort reasons
# after the manual operator abort, corresponds to individual bits of param FW_LND_ABORT
uint8 kNotAborted = 0
uint8 kAbortedByOperator = 1
uint8 kTerrainNotFound = 2 # FW_LND_ABORT (1 << 0)
uint8 kTerrainTimeout = 3 # FW_LND_ABORT (1 << 1)
uint8 kUnknownAbortCriterion = 4
