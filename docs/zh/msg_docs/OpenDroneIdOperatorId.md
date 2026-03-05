---
pageClass: is-wide-page
---

# OpenDroneIdOperatorId (UORB message)

**TOPICS:** open_droneid_operatorid

## Fields

| 参数名                                                        | 类型          | Unit [Frame] | Range/Enum | 描述 |
| ---------------------------------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | -- |
| timestamp                                                  | `uint64`    |                                                                  |            |    |
| id_or_mac        | `uint8[20]` |                                                                  |            |    |
| operator_id_type | `uint8`     |                                                                  |            |    |
| operator_id                           | `char[20]`  |                                                                  |            |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OpenDroneIdOperatorId.msg)

:::details
Click here to see original file

```c
uint64 timestamp
uint8[20] id_or_mac
uint8 operator_id_type
char[20] operator_id
```

:::
