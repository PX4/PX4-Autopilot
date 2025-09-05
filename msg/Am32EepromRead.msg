uint64 timestamp    # time since system start (microseconds)
uint8 index         # Index of the ESC (0 = ESC1, 1 = ESC2, etc.)
uint8[48] data      # Raw AM32 EEPROM data

uint8 ORB_QUEUE_LENGTH = 8 # To support 8 queued up reponses
