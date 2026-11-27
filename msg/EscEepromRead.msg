uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Time the dump was read from the ESC. Unchanged when a cached dump is republished
uint8 firmware # [-] ESC firmware type (see ESC_FIRMWARE enum in MAVLink)
uint8 index # [-] Index of the ESC (0 = ESC1, 1 = ESC2, etc.)
uint16 length # [-] Length of valid data
uint8[48] data # [-] Raw ESC EEPROM data

uint8 ORB_QUEUE_LENGTH = 12 # One dump per ESC, up to esc_status CONNECTED_ESC_MAX
