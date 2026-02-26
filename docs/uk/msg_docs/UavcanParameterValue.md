---
pageClass: is-wide-page
---

# UavcanParameterValue (Параметр UORB)

UAVCAN-MAVLink parameter bridge response type.

**TOPICS:** uavcan_parametervalue

## Fields

| Назва                            | Тип        | Unit [Frame] | Range/Enum | Опис                                                           |
| -------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------- |
| timestamp                        | `uint64`   |                                                                  |            | time since system start (microseconds)      |
| node_id     | `uint8`    |                                                                  |            | UAVCAN node ID mapped from MAVLink component ID                |
| param_id    | `char[17]` |                                                                  |            | MAVLink/UAVCAN parameter name                                  |
| param_index | `int16`    |                                                                  |            | parameter index, if known                                      |
| param_count | `uint16`   |                                                                  |            | number of parameters exposed by the node                       |
| param_type  | `uint8`    |                                                                  |            | MAVLink parameter type                                         |
| int_value   | `int64`    |                                                                  |            | current value if param_type is int-like   |
| real_value  | `float32`  |                                                                  |            | current value if param_type is float-like |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/UavcanParameterValue.msg)

:::details
Click here to see original file

```c
# UAVCAN-MAVLink parameter bridge response type
uint64 timestamp		# time since system start (microseconds)
uint8 node_id			# UAVCAN node ID mapped from MAVLink component ID
char[17] param_id		# MAVLink/UAVCAN parameter name
int16 param_index		# parameter index, if known
uint16 param_count		# number of parameters exposed by the node
uint8 param_type		# MAVLink parameter type
int64 int_value			# current value if param_type is int-like
float32 real_value		# current value if param_type is float-like
```

:::
