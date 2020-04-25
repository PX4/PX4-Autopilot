# Multicopter Control Chain: used to select which Position, Attitude or Rate controller will be used during the mission
# Paired with mavlink chain
uint64 timestamp;

uint8 MC_DISABLE_CONTROLLER = 0
uint8 MC_DEFAULT_CONTROLLER = 1

# Set of custom controllers
# Position controllers: 1xx
uint8 MC_POS_CUSTOM_CONTROLLER = 100

# Attitude controllers: 2xx
uint8 MC_POS_CUSTOM_CONTROLLER = 200

# Rate controllers:	3xx
uint8 MC_POS_CUSTOM_CONTROLLER = 300

uint8 pos_ctl;
uint8 att_ctl;
uint8 rate_ctl;
