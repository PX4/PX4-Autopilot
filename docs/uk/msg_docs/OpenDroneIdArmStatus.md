---
pageClass: is-wide-page
---

# OpenDroneIdArmStatus (UORB message)

**TOPICS:** open_droneid_armstatus

## Fields

| Назва     | Тип        | Unit [Frame] | Range/Enum | Опис |
| --------- | ---------- | ---------------------------------------------------------------- | ---------- | ---- |
| timestamp | `uint64`   |                                                                  |            |      |
| status    | `uint8`    |                                                                  |            |      |
| error     | `char[50]` |                                                                  |            |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OpenDroneIdArmStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp
uint8 status
char[50] error
```

:::
