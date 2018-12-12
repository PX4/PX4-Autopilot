uint64 timestamp		# time since system start (microseconds)
int16[12] channel_id		# ADC channel IDs, negative for non-existent
float32[12] channel_value	# ADC channel value in volt, valid if channel ID is positive
