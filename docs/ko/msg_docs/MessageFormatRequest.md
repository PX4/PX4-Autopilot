---
pageClass: is-wide-page
---

# MessageFormatRequest (UORB message)

**TOPICS:** message_formatrequest

## Fields

| 명칭                                    | 형식         | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                         |
| ------------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`   |                                                                  |            | time since system start (microseconds)                                                                                                                  |
| protocol_version | `uint16`   |                                                                  |            | Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp |
| topic_name       | `char[50]` |                                                                  |            | 예: /fmu/in/vehicle_command                                                                                                            |

## Constants

| 명칭                                                                                                        | 형식       | Value | 설명                                                                                                                                                  |
| --------------------------------------------------------------------------------------------------------- | -------- | ----- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#LATEST_PROTOCOL_VERSION"></a> LATEST_PROTOCOL_VERSION | `uint16` | 1     | Current version of this protocol. Increase this whenever the MessageFormatRequest or MessageFormatResponse changes. |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MessageFormatRequest.msg)

:::details
Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)

# Request to PX4 to get the hash of a message, to check for message compatibility

uint16 LATEST_PROTOCOL_VERSION = 1 # Current version of this protocol. Increase this whenever the MessageFormatRequest or MessageFormatResponse changes.

uint16 protocol_version           # Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp

char[50] topic_name  # E.g. /fmu/in/vehicle_command
```

:::
