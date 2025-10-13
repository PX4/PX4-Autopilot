uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

uint8 NAME_LENGTH = 16
uint8 CHANNELS = 8
uint8 BANDS = 8
uint8 LEVELS = 8

# Configuration name in ASCII
uint8[16] name

# Frequency is in MHz
# Invalid frequencies are set to 0
uint16[8] frequency0
uint16[8] frequency1
uint16[8] frequency2
uint16[8] frequency3
uint16[8] frequency4
uint16[8] frequency5
uint16[8] frequency6
uint16[8] frequency7


# ASCII
uint8[16] band0
uint8[16] band1
uint8[16] band2
uint8[16] band3
uint8[16] band4
uint8[16] band5
uint8[16] band6
uint8[16] band7

# ASCII
uint8[8] letter

# Positive values are the exact value the VTX protocol expects to set the power level
# Negative values are alternative values that map to the same power level like dBm
int16[8] power
# The power output in mW for each power level used for display purposes
uint16[8] power_mw

# All ≤8
uint8 bands
uint8 channels
uint8 levels
