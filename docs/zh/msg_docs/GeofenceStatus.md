---
pageClass: is-wide-page
---

# GeofenceStatus (UORB message)

**TOPICS:** geofence_status

## Fields

| 参数名                              | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| -------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                        | `uint64` |                                                                  |            | time since system start (microseconds) |
| geofence_id | `uint32` |                                                                  |            | loaded geofence id                                        |
| status                           | `uint8`  |                                                                  |            | Current geofence status                                   |

## Constants

| 参数名                                                                                           | 类型      | 值 | 描述 |
| --------------------------------------------------------------------------------------------- | ------- | - | -- |
| <a href="#GF_STATUS_LOADING"></a> GF_STATUS_LOADING | `uint8` | 0 |    |
| <a href="#GF_STATUS_READY"></a> GF_STATUS_READY     | `uint8` | 1 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GeofenceStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp                        # time since system start (microseconds)

uint32 geofence_id 			# loaded geofence id
uint8 status 				# Current geofence status

uint8 GF_STATUS_LOADING = 0
uint8 GF_STATUS_READY = 1
```

:::
