---
pageClass: is-wide-page
---

# PowerButtonState (UORB повідомлення)

power button state notification message.

**TOPICS:** power_buttonstate

## Fields

| Назва     | Тип      | Unit [Frame] | Range/Enum | Опис                                                      |
| --------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64` |                                                                  |            | time since system start (microseconds) |
| event     | `uint8`  |                                                                  |            | one of PWR_BUTTON_STATE_\*           |

## Constants

| Назва                                                                                                                                                                   | Тип     | Значення | Опис                                                                                       |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | -------- | ------------------------------------------------------------------------------------------ |
| <a href="#PWR_BUTTON_STATE_IDEL"></a> PWR_BUTTON_STATE_IDEL                                              | `uint8` | 0        | Button went up without meeting shutdown button down time (delete event) |
| <a href="#PWR_BUTTON_STATE_DOWN"></a> PWR_BUTTON_STATE_DOWN                                              | `uint8` | 1        | Button went Down                                                                           |
| <a href="#PWR_BUTTON_STATE_UP"></a> PWR_BUTTON_STATE_UP                                                  | `uint8` | 2        | Button went Up                                                                             |
| <a href="#PWR_BUTTON_STATE_REQUEST_SHUTDOWN"></a> PWR_BUTTON_STATE_REQUEST_SHUTDOWN | `uint8` | 3        | Button went Up after meeting shutdown button down time                                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PowerButtonState.msg)

:::details
Click here to see original file

```c
# power button state notification message

uint64 timestamp			    # time since system start (microseconds)

uint8 PWR_BUTTON_STATE_IDEL = 0             # Button went up without meeting shutdown button down time (delete event)
uint8 PWR_BUTTON_STATE_DOWN = 1             # Button went Down
uint8 PWR_BUTTON_STATE_UP = 2               # Button went Up
uint8 PWR_BUTTON_STATE_REQUEST_SHUTDOWN = 3 # Button went Up after meeting shutdown button down time

uint8 event                                 # one of PWR_BUTTON_STATE_*
```

:::
