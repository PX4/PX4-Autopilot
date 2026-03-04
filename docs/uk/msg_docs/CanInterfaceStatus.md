---
pageClass: is-wide-page
---

# CanInterfaceStatus (повідомлення UORB)

**TOPICS:** can_interfacestatus

## Fields

| Назва                          | Тип      | Unit [Frame] | Range/Enum | Опис                                                      |
| ------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                      | `uint64` |                                                                  |            | time since system start (microseconds) |
| інтерфейс                      | `uint8`  |                                                                  |            |                                                           |
| io_errors | `uint64` |                                                                  |            |                                                           |
| frames_tx | `uint64` |                                                                  |            |                                                           |
| frames_rx | `uint64` |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CanInterfaceStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
uint8 interface

uint64 io_errors
uint64 frames_tx
uint64 frames_rx
```

:::
