uint64 timestamp		# time since system start (microseconds)

uint32 device_id                # unique device ID for the sensor that does not change between power cycles

uint16 len                       # length of data
uint8 flags                     # LSB: 1=fragmented
uint8[300] data                 # data to write to GPS device (RTCM message)

uint8 ORB_QUEUE_LENGTH = 8

uint8 MAX_INSTANCES = 2
