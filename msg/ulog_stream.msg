# Message to stream ULog data from the logger. Corresponds to the LOGGING_DATA
# mavlink message

# flags bitmasks
uint8 FLAGS_NEED_ACK = 1     # if set, this message requires to be acked.
                             # Acked messages are published synchronous: a
                             # publisher waits for an ack before sending the
                             # next message

uint8 length                 # length of data
uint8 first_message_offset   # offset into data where first message starts. This
                             # can be used for recovery, when a previous message got lost
uint16 sequence              # allows determine drops
uint8 flags                  # see FLAGS_*
uint8[249] data              # ulog data

