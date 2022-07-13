uint64 timestamp # [us] time since system start
float32 lateral_touchdown_offset # [m] lateral touchdown position offset manually commanded during landing
bool flaring # true if the aircraft is flaring
bool abort_landing # true if landing should be aborted
int32 abort_reason # the singular abort criterion which triggered the landing abort

# abort reasons
# corresponds to individual bits of param FW_LND_ABORT
uint8 kAbortReasonNone = 0
uint8 kAbortReasonTerrainNotFound = 1 # (1 << 0)
uint8 kAbortReasonTerrainTimeout = 2 # (1 << 1)
