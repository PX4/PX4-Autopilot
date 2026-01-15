# ParameterListRequest: Request to enumerate parameters
# Allows paginated listing of all parameters or only used parameters

uint64 timestamp                # time since system start (microseconds)

uint16 start_index              # Starting index for pagination (0-based)
uint8 count                     # Number of parameters to return (max 10)

bool used_only                  # If true, only return parameters marked as "used"

uint8 MAX_COUNT = 10            # Maximum parameters per response

uint8 ORB_QUEUE_LENGTH = 4
