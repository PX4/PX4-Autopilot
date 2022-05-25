# LED States for El Mandadero and NXP Buggy3

uint64 timestamp

uint8 IDLE     = 0 # Dim Red
uint8 BRAKE    = 1 # Bright Red
uint8 REVERSE  = 2 # [Top] White
uint8 TURN     = 3 # [Top] Flashing Yellow
uint8 HAZARD   = 4 # Flashing Yellow
uint8 NO_STATE = 255 # No state update

# LED Panel IDs
# 0 - Right Front
# 1 - Left Front
# 2 - Left Rear
# 3 - Right Rear
uint8[10] state
