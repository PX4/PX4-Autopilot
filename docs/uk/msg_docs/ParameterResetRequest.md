---
pageClass: is-wide-page
---

# ParameterResetRequest (повідомлення UORB)

ParameterResetRequest : Used by the primary to reset one or all parameter value(s) on the remote.

**TOPICS:** parameter_resetrequest

## Fields

| Назва                                | Тип      | Unit [Frame] | Range/Enum | Опис                                                             |
| ------------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------- |
| timestamp                            | `uint64` |                                                                  |            |                                                                  |
| parameter_index | `uint16` |                                                                  |            |                                                                  |
| reset_all       | `bool`   |                                                                  |            | If this is true then ignore parameter_index |

## Constants

| Назва                                                                                       | Тип     | Значення | Опис |
| ------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ParameterResetRequest.msg)

:::details
Click here to see original file

```c
# ParameterResetRequest : Used by the primary to reset one or all parameter value(s) on the remote

uint64 timestamp
uint16 parameter_index

bool reset_all              # If this is true then ignore parameter_index

uint8 ORB_QUEUE_LENGTH = 4
```

:::
