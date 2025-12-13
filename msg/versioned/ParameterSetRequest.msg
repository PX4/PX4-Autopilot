uint32 MESSAGE_VERSION = 0

uint64 timestamp

int16 param_index     		# index of the parameter to set, -1 to use param_id
char[16] param_id		# name of the parameter to set, used if param_index is -1

uint8 TYPE_INT32 = 1
uint8 TYPE_FLOAT = 2
uint8 param_type

int32 int_value
float32 real_value

uint8 ORB_QUEUE_LENGTH = 32

