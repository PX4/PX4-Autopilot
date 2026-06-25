---
pageClass: is-wide-page
---

# DebugKeyValue (UORB message)

**TOPICS:** debug_key_value

## Fields

| Name                                | Type       | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------- | ---------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`   |              |            | time since system start (microseconds) |
| <a id="fld_key"></a>key             | `char[10]` |              |            | max. 10 characters as key / name       |
| <a id="fld_value"></a>value         | `float32`  |              |            | the value to send as debug output      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugKeyValue.msg)

::: details Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
char[10] key			# max. 10 characters as key / name
float32 value			# the value to send as debug output
```

:::
