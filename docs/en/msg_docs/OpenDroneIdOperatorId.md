---
pageClass: is-wide-page
---

# OpenDroneIdOperatorId (UORB message)

**TOPICS:** open_drone_id_operator_id

## Fields

| Name                                              | Type        | Unit [Frame] | Range/Enum | Description |
| ------------------------------------------------- | ----------- | ------------ | ---------- | ----------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`    |              |            |
| <a id="fld_id_or_mac"></a>id_or_mac               | `uint8[20]` |              |            |
| <a id="fld_operator_id_type"></a>operator_id_type | `uint8`     |              |            |
| <a id="fld_operator_id"></a>operator_id           | `char[20]`  |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OpenDroneIdOperatorId.msg)

::: details Click here to see original file

```c
uint64 timestamp
uint8[20] id_or_mac
uint8 operator_id_type
char[20] operator_id
```

:::
