# This message is used to dump the raw gps communication to the log.

uint64 timestamp # time since system start (microseconds)

uint8 INSTANCE_MAIN = 0
uint8 INSTANCE_SECONDARY = 1

uint8 instance   # Instance of GNSS receiver
uint32 device_id
uint8 len        # length of data, MSB bit set = message to the gps device,
                 # clear = message from the device
uint8[79] data   # data to write to the log

uint8 ORB_QUEUE_LENGTH = 16
