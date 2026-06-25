---
pageClass: is-wide-page
---

# DebugArray (повідомлення UORB)

**TOPICS:** debug_array

## Fields

| Назва                               | Тип           | Unit [Frame] | Range/Enum | Опис                                                                            |
| ----------------------------------- | ------------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`      |                                                                  |            | time since system start (microseconds)                       |
| <a id="fld_id"></a>id               | `uint16`      |                                                                  |            | unique ID of debug array, used to discriminate between arrays                   |
| <a id="fld_name"></a>name           | `char[10]`    |                                                                  |            | name of the debug array (max. 10 characters) |
| <a id="fld_data"></a>data           | `float32[58]` |                                                                  |            | data                                                                            |

## Constants

| Назва                                                    | Тип     | Значення | Опис |
| -------------------------------------------------------- | ------- | -------- | ---- |
| <a id="#ARRAY_SIZE"></a> ARRAY_SIZE | `uint8` | 58       |      |

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
