# A logging message, output with PX4_WARN, PX4_ERR, PX4_INFO

uint64 timestamp		# time since system start (microseconds)

uint8 severity # log level (same as in the linux kernel, starting with 0)
char[127] text

uint8 ORB_QUEUE_LENGTH = 4
