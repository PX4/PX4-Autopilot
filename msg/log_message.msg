# A logging message, output with PX4_{WARN,ERR,INFO}

uint64 timestamp		# time since system start (microseconds)

uint8 severity # log level (same as in the linux kernel, starting with 0)
uint8[127] text
