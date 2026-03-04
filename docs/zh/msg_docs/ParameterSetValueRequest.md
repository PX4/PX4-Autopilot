---
pageClass: is-wide-page
---

# ParameterSetValueRequest (UORB message)

ParameterSetValueRequest : Used by a remote or primary to update the value for a parameter at the other end.

**TOPICS:** parameter_set_value_request parameter_remote_set_value_request parameter_primary_set_value_request

## Fields

| 参数名                                  | 类型        | Unit [Frame] | Range/Enum | 描述                                      |
| ------------------------------------ | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------- |
| timestamp                            | `uint64`  |                                                                  |            |                                         |
| parameter_index | `uint16`  |                                                                  |            |                                         |
| int_value       | `int32`   |                                                                  |            | Optional value for an integer parameter |
| float_value     | `float32` |                                                                  |            | Optional value for a float parameter    |

## Constants

| 参数名                                                                                         | 类型      | 值  | 描述 |
| ------------------------------------------------------------------------------------------- | ------- | -- | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 32 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ParameterSetValueRequest.msg)

:::details
Click here to see original file

```c
# ParameterSetValueRequest : Used by a remote or primary to update the value for a parameter at the other end

uint64 timestamp
uint16 parameter_index

int32 int_value             # Optional value for an integer parameter
float32 float_value         # Optional value for a float parameter

uint8 ORB_QUEUE_LENGTH = 32

# TOPICS parameter_set_value_request parameter_remote_set_value_request parameter_primary_set_value_request
```

:::
