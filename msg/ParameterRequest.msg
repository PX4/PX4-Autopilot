# parameter request type
uint64 timestamp		# time since system start (microseconds)

uint8 MESSAGE_TYPE_INVALID            = 0
uint8 MESSAGE_TYPE_PARAM_REQUEST_READ = 1
uint8 MESSAGE_TYPE_PARAM_REQUEST_LIST = 2
uint8 MESSAGE_TYPE_PARAM_SET          = 3
uint8 message_type                        # message type

uint8 NODE_ID_ALL = 0		# MAV_COMP_ID_ALL
uint8 node_id			# node ID

char[17] name                   # parameter name
int16 param_index		# -1 if the param_id field should be used as identifier

uint8 TYPE_INVALID = 0
uint8 TYPE_BOOL    = 1
uint8 TYPE_UINT8   = 2
uint8 TYPE_INT32   = 3
uint8 TYPE_INT64   = 4
uint8 TYPE_FLOAT32 = 5
uint8 TYPE_FLOAT64 = 6
uint8 type                # parameter type

int64 int64_value               # current value if param_type is int-like
float64 float64_value           # current value if param_type is float-like

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS parameter_request uavcan_parameter_request
