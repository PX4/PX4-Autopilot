uint64 timestamp  # time since system start (microseconds)
uint8 index       # The index of the ESC
uint8[128] data   # Raw EEPROM data (ESC settings)
uint8 length 	  # Lenth of data

uint8 ORB_QUEUE_LENGTH = 4 # To support 4 queued up reponses
