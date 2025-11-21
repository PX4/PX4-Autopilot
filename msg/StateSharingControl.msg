uint64 timestamp

# Command enum
uint8 COMMAND_NONE = 0
uint8 COMMAND_START = 1
uint8 COMMAND_STOP = 2
uint8 COMMAND_UPDATE_PARAMS = 3

uint8 command

# Optional arguments (key-value pairs: "SHARING_PERIOD=1.0,DELAY_START=0.5")
char[128] args

# TOPICS incoming_state_sharing_control outgoing_state_sharing_control
