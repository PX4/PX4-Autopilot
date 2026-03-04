---
pageClass: is-wide-page
---

# ParameterSetValueResponse (UORB message)

ParameterSetValueResponse : Response to a set value request by either primary or secondary.

**TOPICS:** parameter_set_value_response parameter_remote_set_value_response parameter_primary_set_value_response

## Fields

| 명칭                                     | 형식       | Unit [Frame] | Range/Enum | 설명 |
| -------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | -- |
| timestamp                              | `uint64` |                                                                  |            |    |
| request_timestamp | `uint64` |                                                                  |            |    |
| parameter_index   | `uint16` |                                                                  |            |    |

## Constants

| 명칭                                                                                          | 형식      | Value | 설명 |
| ------------------------------------------------------------------------------------------- | ------- | ----- | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ParameterSetValueResponse.msg)

:::details
Click here to see original file

```c
# ParameterSetValueResponse : Response to a set value request by either primary or secondary

uint64 timestamp
uint64 request_timestamp
uint16 parameter_index

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS parameter_set_value_response parameter_remote_set_value_response parameter_primary_set_value_response
```

:::
