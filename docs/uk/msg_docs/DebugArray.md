---
pageClass: is-wide-page
---

# DebugArray (повідомлення UORB)

**TOPICS:** debug_array

## Fields

| Назва     | Тип           | Unit [Frame] | Range/Enum | Опис                                                                            |
| --------- | ------------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------- |
| timestamp | `uint64`      |                                                                  |            | time since system start (microseconds)                       |
| id        | `uint16`      |                                                                  |            | unique ID of debug array, used to discriminate between arrays                   |
| name      | `char[10]`    |                                                                  |            | name of the debug array (max. 10 characters) |
| data      | `float32[58]` |                                                                  |            | data                                                                            |

## Constants

| Назва                                                      | Тип     | Значення | Опис |
| ---------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#ARRAY_SIZE"></a> ARRAY_SIZE | `uint8` | 58       |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugArray.msg)

:::details
Click here to see original file

```c
uint8 ARRAY_SIZE = 58
uint64 timestamp            # time since system start (microseconds)
uint16 id                   # unique ID of debug array, used to discriminate between arrays
char[10] name               # name of the debug array (max. 10 characters)
float32[58] data            # data
```

:::
