---
pageClass: is-wide-page
---

# DebugKeyValue (повідомлення UORB)

**TOPICS:** debug_keyvalue

## Fields

| Назва     | Тип        | Unit [Frame] | Range/Enum | Опис                                                      |
| --------- | ---------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64`   |                                                                  |            | time since system start (microseconds) |
| key       | `char[10]` |                                                                  |            | max. 10 characters as key / name          |
| value     | `float32`  |                                                                  |            | the value to send as debug output                         |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugKeyValue.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
char[10] key			# max. 10 characters as key / name
float32 value			# the value to send as debug output
```

:::
