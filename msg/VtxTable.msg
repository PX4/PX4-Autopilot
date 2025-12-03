uint64 timestamp # time since system start (microseconds)

uint8 NAME_LENGTH = 16
uint8 LABEL_LENGTH = 4
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


# Band names in ASCII
uint8[16] band0
uint8[16] band1
uint8[16] band2
uint8[16] band3
uint8[16] band4
uint8[16] band5
uint8[16] band6
uint8[16] band7

# Band letters in ASCII
uint8[8] letter

# Band attributes
uint8 ATTRIBUTE_FACTORY = 0
uint8 ATTRIBUTE_CUSTOM = 1
uint8[8] attribute

# Positive values are the exact value the VTX protocol expects to set the power level
# Negative values are alternative values depending on the VTX protocol and device
int16[8] power
# The power labels as ASCII
uint8[4] power_label0
uint8[4] power_label1
uint8[4] power_label2
uint8[4] power_label3
uint8[4] power_label4
uint8[4] power_label5
uint8[4] power_label6
uint8[4] power_label7

# All ≤8
uint8 bands
uint8 channels
uint8 levels
