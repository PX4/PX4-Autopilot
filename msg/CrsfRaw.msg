# Raw CRSF frame data for logging/debugging/bridging
# crsf_raw_rx: Data received from serial port (CRSF receiver)
# crsf_raw_tx: Data to be transmitted to serial port (CRSF receiver)

uint64 timestamp                    # time since system start (microseconds)

uint8 len                           # Length of data
uint8[64] data                      # Raw CRSF frame data

uint8 ORB_QUEUE_LENGTH = 8

# TOPICS crsf_raw_rx crsf_raw_tx
