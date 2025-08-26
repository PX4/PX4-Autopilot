uint64 timestamp     # time since system start (microseconds)
uint8 index          # Index of the ESC (0 = ESC1, 1 = ESC2, etc.)
uint8[48] data       # Raw AM32 EEPROM data
uint32[2] write_mask # Bitmask indicating which bytes in the data array should be written (max 64 values, am32 is currently 48)

uint8 ORB_QUEUE_LENGTH = 4 # To support 4 queued up reponses
