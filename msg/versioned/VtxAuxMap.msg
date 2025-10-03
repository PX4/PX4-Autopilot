uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

# See https://betaflight.com/docs/wiki/guides/current/VTX-CLI-Settings#change-vtx-power-level-using-aux-channel

uint8 MAX_LENGTH = 32

uint8 length
int8[32] aux_channel
int8[32] band
int8[32] channel
int8[32] power_level
int16[32] start_range
int16[32] end_range
