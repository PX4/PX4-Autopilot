# ParameterListResponse: Response containing a list of parameters
# Returns paginated parameter information including names, types, and values

uint64 timestamp                # time since system start (microseconds)
uint64 request_timestamp        # timestamp from the request (for correlation)

uint16 total_count              # Total number of parameters (or total used if used_only was set)
uint8 returned_count            # Number of parameters in this response (0-10)
uint16 start_index              # Starting index echoed back

# Parameter data arrays (up to 10 parameters per response)
uint16[10] parameter_indices    # Parameter indices
char[170] param_names           # Packed parameter names (10 x 17 chars)
uint8[10] param_types           # Parameter types (1=INT32, 2=FLOAT)
int32[10] int_values            # Integer values (valid when param_type=INT32)
float32[10] float_values        # Float values (valid when param_type=FLOAT)

# Parameter type constants (prefixed to avoid conflict with param.h macros)
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_INT32 = 1
uint8 TYPE_FLOAT = 2

uint8 MAX_COUNT = 10            # Maximum parameters per response
uint8 PARAM_NAME_LEN = 17       # Length of each parameter name slot

uint8 ORB_QUEUE_LENGTH = 4
