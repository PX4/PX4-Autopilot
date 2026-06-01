---
pageClass: is-wide-page
---

# OpenDroneIdSelfId (UORB message)

**TOPICS:** open_drone_id_self_id

## Fields

| Name                                              | Type        | Unit [Frame] | Range/Enum | Description |
| ------------------------------------------------- | ----------- | ------------ | ---------- | ----------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`    |              |            |
| <a id="fld_id_or_mac"></a>id_or_mac               | `uint8[20]` |              |            |
| <a id="fld_description_type"></a>description_type | `uint8`     |              |            |
| <a id="fld_description"></a>description           | `char[23]`  |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OpenDroneIdSelfId.msg)

::: details Click here to see original file

```c
uint64 timestamp
uint8[20] id_or_mac
uint8 description_type
char[23] description
```

:::
