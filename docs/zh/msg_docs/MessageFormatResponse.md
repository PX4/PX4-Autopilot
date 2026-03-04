---
pageClass: is-wide-page
---

# MessageFormatResponse (UORB message)

**TOPICS:** message_formatresponse

## Fields

| 参数名                                   | 类型         | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                         |
| ------------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`   |                                                                  |            | time since system start (microseconds)                                                                                                                  |
| protocol_version | `uint16`   |                                                                  |            | Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp |
| topic_name       | `char[50]` |                                                                  |            | E.g. /fmu/in/vehicle_command                                                                                          |
| success                               | `bool`     |                                                                  |            |                                                                                                                                                                            |
| message_hash     | `uint32`   |                                                                  |            | hash over all message fields                                                                                                                                               |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MessageFormatResponse.msg)

:::details
Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)

# Response from PX4 with the format of a message

uint16 protocol_version           # Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp

char[50] topic_name  # E.g. /fmu/in/vehicle_command

bool success
uint32 message_hash # hash over all message fields
```

:::
