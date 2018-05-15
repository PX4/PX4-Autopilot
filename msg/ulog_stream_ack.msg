# Ack a previously sent ulog_stream message that had
# the NEED_ACK flag set

int32 ACK_TIMEOUT = 50         # timeout waiting for an ack until we retry to send the message [ms]
int32 ACK_MAX_TRIES = 50         # maximum amount of tries to (re-)send a message, each time waiting ACK_TIMEOUT ms

uint16 sequence
