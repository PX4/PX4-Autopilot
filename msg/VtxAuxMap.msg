uint64 timestamp # time since system start (microseconds)

# See https://betaflight.com/docs/wiki/guides/current/VTX-CLI-Settings#change-vtx-power-level-using-aux-channel

uint8 MAX_LENGTH = 160

uint8 length
int8[160] aux_channel
int8[160] band
int8[160] channel
int8[160] power_level
int16[160] start_range
int16[160] end_range
