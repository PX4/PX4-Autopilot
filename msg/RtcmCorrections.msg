uint64 timestamp		# time since system start (microseconds)

uint32 device_id                # unique device ID of the publisher that produced this RTCM

uint16 len                      # length of data
uint8 flags                     # LSB: 1=fragmented
uint8[300] data                 # RTCM3 correction data (from a fixed base, or another GPS)

uint8 ORB_QUEUE_LENGTH = 8

uint8 MAX_INSTANCES = 4
