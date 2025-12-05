uint32 MESSAGE_VERSION = 0

uint64 timestamp
int16 param_index     # index of the parameter to get, -1 to use param_id
char[16] param_id     # name of the parameter to get, used if param_index is -1

uint8 ORB_QUEUE_LENGTH = 64

