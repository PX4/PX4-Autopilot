uint32 MESSAGE_VERSION = 0

uint64 timestamp

uint8 TYPE_INT32 = 1
uint8 TYPE_FLOAT = 2
uint8 param_type

int32 int_value
float32 real_value

int16 param_index		# index of the returned parameter
char[16] param_id		# name of the returned parameter

bool success			# false if the parameter doesn't exist or couldn't be set

uint8 ORB_QUEUE_LENGTH = 4

