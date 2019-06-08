uint64 timestamp

uint8 type

uint8 LOGGER_TYPE_FULL = 0       # no battery low voltage warning active
uint8 LOGGER_TYPE_MISSION = 1        # warning of low voltage

float32 written_kb			# current write rate in bytes
float32 write_rate_kb_s			# write rate in kB/s

uint32 dropouts

uint32 buffer_size_bytes
uint32 buffer_used_bytes

uint8 num_messages
