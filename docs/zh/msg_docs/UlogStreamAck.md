---
pageClass: is-wide-page
---

# UlogStreamAck (UORB message)

Ack a previously sent ulog_stream message that had. the NEED_ACK flag set.

**TOPICS:** ulog_streamack

## Fields

| 参数名                               | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| --------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                         | `uint64` |                                                                  |            | time since system start (microseconds) |
| msg_sequence | `uint16` |                                                                  |            |                                                           |

## Constants

| 参数名                                                                                   | 类型      | 值  | 描述                                                                                                                       |
| ------------------------------------------------------------------------------------- | ------- | -- | ------------------------------------------------------------------------------------------------------------------------ |
| <a href="#ACK_TIMEOUT"></a> ACK_TIMEOUT                          | `int32` | 50 | timeout waiting for an ack until we retry to send the message [ms]   |
| <a href="#ACK_MAX_TRIES"></a> ACK_MAX_TRIES | `int32` | 50 | maximum amount of tries to (re-)send a message, each time waiting ACK_TIMEOUT ms |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/UlogStreamAck.msg)

:::details
Click here to see original file

```c
# Ack a previously sent ulog_stream message that had
# the NEED_ACK flag set

uint64 timestamp		# time since system start (microseconds)
int32 ACK_TIMEOUT = 50		# timeout waiting for an ack until we retry to send the message [ms]
int32 ACK_MAX_TRIES = 50	# maximum amount of tries to (re-)send a message, each time waiting ACK_TIMEOUT ms

uint16 msg_sequence
```

:::
