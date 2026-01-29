# ParameterNameResponse: Response to a parameter name lookup request
# Returns parameter index, type, and current value

uint64 timestamp                # time since system start (microseconds)
uint64 request_timestamp        # timestamp from the request (for correlation)

char[17] param_name             # Parameter name echoed back
uint16 parameter_index          # Parameter index (0xFFFF = PARAM_INVALID if not found)

uint8 param_type                # Parameter type (1=INT32, 2=FLOAT, 0=UNKNOWN)
int32 int_value                 # Value if param_type is INT32
float32 float_value             # Value if param_type is FLOAT

bool valid                      # True if parameter was found

# Parameter type constants (prefixed to avoid conflict with param.h macros)
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_INT32 = 1
uint8 TYPE_FLOAT = 2

uint8 ORB_QUEUE_LENGTH = 8
