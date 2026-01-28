uint64 timestamp # time since system start (microseconds)

uint8 BAND_NAME_LENGTH = 12
uint8 POWER_LABEL_LENGTH = 4

uint8 PROTOCOL_NONE = 0              # No protocol is detected, usually an error
uint8 PROTOCOL_SMART_AUDIO_V1 = 10
uint8 PROTOCOL_SMART_AUDIO_V2 = 20
uint8 PROTOCOL_SMART_AUDIO_V2_1 = 21
uint8 PROTOCOL_TRAMP = 100
uint8 protocol

uint8 DEVICE_UNKNOWN = 0
uint8 DEVICE_PEAK_THOR_T67 = 20
uint8 DEVICE_RUSH_MAX_SOLO = 40
uint8 device

uint8 MODE_NORMAL = 0
uint8 MODE_PIT = 1
uint8 mode

# Band and Channel are 0-indexed! But the user expects a 1-indexed display!
int8 band        # Band number (0-23), negative values indicate frequency mode
int8 channel     # Channel number (0-15), negative values indicate frequency mode
uint16 frequency # Frequency in MHz, zero indicates unknown

uint8 band_letter   # Band letter as ASCII
uint8[12] band_name # Band name in ASCII without null termination

# Also 0-indexed, but the user expects a 1-indexed display!
int8 power_level     # Current power level (0-15), negative values indicate unknown
uint8[4] power_label # Current power label in ASCII without null termination
