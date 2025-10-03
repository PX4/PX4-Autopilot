uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

uint8 PROTOCOL_NONE = 0              # No protocol is detected, this is usually an error
uint8 PROTOCOL_SMART_AUDIO_V1 = 10
uint8 PROTOCOL_SMART_AUDIO_V2 = 20
uint8 PROTOCOL_SMART_AUDIO_V2_1 = 21
uint8 PROTOCOL_TRAMP = 100
uint8 protocol

uint8 MODE_NORMAL = 0
uint8 MODE_PIT = 1
uint8 mode

# Band and Channel are 0-indexed! But the user expects a 1-indexed display!
uint8 band        # Band number (0-7)
uint8 channel     # Channel number (0-7)
uint16 frequency  # Frequency in MHz

# Also 0-indexed, but the user expects a 1-indexed display!
int8 power_level  # Current power level (0-7), negative values indicate unknown
uint16 power_mw   # Current power output in mW, zero indicates unknown
