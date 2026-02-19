---
pageClass: is-wide-page
---

# RcParameterMap (UORB message)

**TOPICS:** rc_parametermap

## Fields

| 명칭                               | 형식           | Unit [Frame] | Range/Enum | 설명                                                                                                                                    |
| -------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                        | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                             |
| valid                            | `bool[3]`    |                                                                  |            | true for RC-Param channels which are mapped to a param                                                                                |
| param_index | `int32[3]`   |                                                                  |            | corresponding param index, this field is ignored if set to -1, in this case param_id will be used                |
| param_id    | `char[51]`   |                                                                  |            | MAP_NCHAN \* (ID_LEN + 1) chars, corresponding param id, null terminated |
| scale                            | `float32[3]` |                                                                  |            | scale to map the RC input [-1, 1] to a parameter value                            |
| value0                           | `float32[3]` |                                                                  |            | initial value around which the parameter value is changed                                                                             |
| value_min   | `float32[3]` |                                                                  |            | minimal parameter value                                                                                                               |
| value_max   | `float32[3]` |                                                                  |            | minimal parameter value                                                                                                               |

## Constants

| 명칭                                                                                                                   | 형식      | Value | 설명                                                                                                                                                                                                           |
| -------------------------------------------------------------------------------------------------------------------- | ------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a href="#RC_PARAM_MAP_NCHAN"></a> RC_PARAM_MAP_NCHAN | `uint8` | 3     | This limit is also hardcoded in the enum RC_CHANNELS_FUNCTION in rc_channels.h                                                |
| <a href="#PARAM_ID_LEN"></a> PARAM_ID_LEN                                  | `uint8` | 16    | corresponds to MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RcParameterMap.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
uint8 RC_PARAM_MAP_NCHAN = 3 # This limit is also hardcoded in the enum RC_CHANNELS_FUNCTION in rc_channels.h
uint8 PARAM_ID_LEN = 16 # corresponds to MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN

bool[3] valid		#true for RC-Param channels which are mapped to a param
int32[3] param_index	# corresponding param index, this field is ignored if set to -1, in this case param_id will be used
char[51] param_id	# MAP_NCHAN * (ID_LEN + 1) chars, corresponding param id, null terminated
float32[3] scale		# scale to map the RC input [-1, 1] to a parameter value
float32[3] value0		# initial value around which the parameter value is changed
float32[3] value_min	# minimal parameter value
float32[3] value_max	# minimal parameter value
```

:::
