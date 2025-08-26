uint64 timestamp    # time since system start (microseconds)
uint8 index         # Index of the ESC (0 = ESC1, 1 = ESC2, etc.)
uint8 length 	    # Number of valid bytes in data array
uint8[192] data     # Raw AM32 EEPROM data

uint8 ORB_QUEUE_LENGTH = 4 # To support 4 queued up reponses
