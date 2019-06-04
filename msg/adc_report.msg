uint64 timestamp		# time since system start (microseconds)

uint8 MAX_CHANNELS = 12

int16[12] channel_id		# ADC channel IDs, negative for non-existent
int16[12] channel_value		# ADC convert result (2 bytes)

uint8 channels
