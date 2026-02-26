---
pageClass: is-wide-page
---

# OpenDroneIdSelfId (UORB message)

**TOPICS:** open_droneid_selfid

## Fields

| Назва                                               | Тип         | Unit [Frame] | Range/Enum | Опис |
| --------------------------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | ---- |
| timestamp                                           | `uint64`    |                                                                  |            |      |
| id_or_mac | `uint8[20]` |                                                                  |            |      |
| description_type               | `uint8`     |                                                                  |            |      |
| description                                         | `char[23]`  |                                                                  |            |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OpenDroneIdSelfId.msg)

:::details
Click here to see original file

```c
uint64 timestamp
uint8[20] id_or_mac
uint8 description_type
char[23] description
```

:::
