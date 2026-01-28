uint64 timestamp     # time since system start (microseconds)
uint8 firmware       # ESC firmware type (see ESC_FIRMWARE enum in MAVLink)
uint8 index          # Index of the ESC (0 = ESC1, 1 = ESC2, etc, 255 = All)
uint16 length	     # Length of valid data
uint8[64] data       # Raw ESC EEPROM data
uint32[2] write_mask # Bitmask indicating which bytes in the data array should be written (max 64 values)

uint8 ORB_QUEUE_LENGTH = 8 # To support 8 queued up requests
