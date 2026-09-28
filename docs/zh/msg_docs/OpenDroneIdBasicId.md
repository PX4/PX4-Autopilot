---
pageClass: is-wide-page
---

# OpenDroneIdBasicId (UORB message)

**TOPICS:** open_drone_id_basic_id

## Fields

| 参数名                                                                           | 类型          | Unit [Frame] | Range/Enum | 描述                                                                                                                                                               |
| ----------------------------------------------------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                           | `uint64`    |                                                                  |            |                                                                                                                                                                  |
| <a id="fld_id_or_mac"></a>id_or_mac | `uint8[20]` |                                                                  |            | Only used for drone ID data received from other UAs, no null termination, null filled if shorter                                                                 |
| <a id="fld_id_type"></a>id_type                          | `uint8`     |                                                                  |            | MAV_ODID_ID_TYPE: indicates the format for the uas_id field  |
| <a id="fld_ua_type"></a>ua_type                          | `uint8`     |                                                                  |            | MAV_ODID_UA_TYPE: indicates the type of UA (Unmanned Aircraft) |
| <a id="fld_uas_id"></a>uas_id                            | `uint8[20]` |                                                                  |            | UAS (Unmanned Aircraft System) ID following the format specified by id_type, no null termination, null filled if shorter |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OpenDroneIdBasicId.msg)

:::details
Click here to see original file

```c
uint64 timestamp
uint8[20] id_or_mac	# Only used for drone ID data received from other UAs, no null termination, null filled if shorter
uint8 id_type		# MAV_ODID_ID_TYPE: indicates the format for the uas_id field
uint8 ua_type		# MAV_ODID_UA_TYPE: indicates the type of UA (Unmanned Aircraft)
uint8[20] uas_id	# UAS (Unmanned Aircraft System) ID following the format specified by id_type, no null termination, null filled if shorter
```

:::
