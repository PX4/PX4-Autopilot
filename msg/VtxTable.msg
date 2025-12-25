uint64 timestamp # time since system start (microseconds)

uint8 NAME_LENGTH = 16
uint8 BAND_NAME_LENGTH = 12
uint8 POWER_LABEL_LENGTH = 4
uint8 CHANNELS = 16
uint8 BANDS = 24
uint8 POWER_LEVELS = 16

# Configuration name in ASCII without null termination
uint8[16] name

uint8 bands
uint8 channels
# Frequency is in MHz
# Invalid frequencies are set to 0
uint16[384] frequency # 24*16 = 384

# Band names in ASCII without null termination
uint8[288] band_name # 24*12 = 288

# Band letters in ASCII
uint8[24] letter

# Band attributes
uint8 ATTRIBUTE_FACTORY = 0
uint8 ATTRIBUTE_CUSTOM = 1
uint8[24] attribute

uint8 power_levels
# Positive values are the exact value the VTX protocol expects to set the power level
# Negative values are alternative values depending on the VTX protocol and device
int16[16] power_value
# The power labels as ASCII without null termination
uint8[64] power_label # 4*16 = 64
