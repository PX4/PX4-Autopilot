---
pageClass: is-wide-page
---

# ParameterSetUsedRequest (UORB message)

ParameterSetUsedRequest : Used by a remote to update the used flag for a parameter on the primary.

**TOPICS:** parameter_setused_request

## Fields

| 参数名                                  | 类型       | Unit [Frame] | Range/Enum | 描述 |
| ------------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | -- |
| timestamp                            | `uint64` |                                                                  |            |    |
| parameter_index | `uint16` |                                                                  |            |    |

## Constants

| 参数名                                                                                         | 类型      | 值  | 描述 |
| ------------------------------------------------------------------------------------------- | ------- | -- | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 64 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ParameterSetUsedRequest.msg)

:::details
Click here to see original file

```c
# ParameterSetUsedRequest : Used by a remote to update the used flag for a parameter on the primary

uint64 timestamp
uint16 parameter_index

uint8 ORB_QUEUE_LENGTH = 64
```

:::
