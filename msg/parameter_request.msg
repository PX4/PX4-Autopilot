# UAVCAN-MAVLink parameter bridge request type
uint64 timestamp                                # time since system start (microseconds)

uint8 MESSAGE_TYPE_PARAM_REQUEST_READ = 20      # MAVLINK_MSG_ID_PARAM_REQUEST_READ
uint8 MESSAGE_TYPE_PARAM_REQUEST_LIST = 21      # MAVLINK_MSG_ID_PARAM_REQUEST_LIST
uint8 MESSAGE_TYPE_PARAM_SET          = 23      # MAVLINK_MSG_ID_PARAM_SET
uint8 message_type                              # message type: PARAM_REQUEST_READ, PARAM_REQUEST_LIST, PARAM_SET

uint8 NODE_ID_ALL = 0           # MAV_COMP_ID_ALL
uint8 node_id                   # UAVCAN node ID mapped from MAVLink component ID

char[17] param_id               # parameter name
int16 param_index               # -1 if the param_id field should be used as identifier

uint8 TYPE_UINT8  = 1
uint8 TYPE_INT32  = 6
uint8 TYPE_INT64  = 8
uint8 TYPE_REAL32 = 9
uint8 param_type                # parameter type

int64 int_value                 # current value if param_type is int-like
float32 real_value              # current value if param_type is float-like

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS parameter_request uavcan_parameter_request
