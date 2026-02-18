---
pageClass: is-wide-page
---

# UlogStream (UORB message)

Message to stream ULog data from the logger. Corresponds to the LOGGING_DATA. mavlink message.

**TOPICS:** ulog_stream

## Fields

| 参数名                                                            | 类型           | Unit [Frame] | Range/Enum | 描述                                                                |
| -------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------- |
| timestamp                                                      | `uint64`     |                                                                  |            | time since system start (microseconds)         |
| length                                                         | `uint8`      |                                                                  |            | length of data                                                    |
| first_message_offset | `uint8`      |                                                                  |            | offset into data where first message starts. This |
| msg_sequence                              | `uint16`     |                                                                  |            | allows determine drops                                            |
| flags                                                          | `uint8`      |                                                                  |            | see FLAGS\_\*                               |
| data                                                           | `uint8[249]` |                                                                  |            | ulog data                                                         |

## Constants

| 参数名                                                                                         | 类型      | 值  | 描述                                                                                   |
| ------------------------------------------------------------------------------------------- | ------- | -- | ------------------------------------------------------------------------------------ |
| <a href="#FLAGS_NEED_ACK"></a> FLAGS_NEED_ACK     | `uint8` | 1  | if set, this message requires to be acked.                           |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 16 | TODO: we might be able to reduce this if mavlink polled on the topic |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/UlogStream.msg)

:::details
Click here to see original file

```c
# Message to stream ULog data from the logger. Corresponds to the LOGGING_DATA
# mavlink message

uint64 timestamp		# time since system start (microseconds)

# flags bitmasks
uint8 FLAGS_NEED_ACK = 1	# if set, this message requires to be acked.
				# Acked messages are published synchronous: a
				# publisher waits for an ack before sending the
				# next message

uint8 length			# length of data
uint8 first_message_offset	# offset into data where first message starts. This
				# can be used for recovery, when a previous message got lost
uint16 msg_sequence		# allows determine drops
uint8 flags			# see FLAGS_*
uint8[249] data		# ulog data

uint8 ORB_QUEUE_LENGTH = 16	# TODO: we might be able to reduce this if mavlink polled on the topic
```

:::
