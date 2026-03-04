---
pageClass: is-wide-page
---

# GeofenceResult (повідомлення UORB)

**TOPICS:** geofence_result

## Fields

| Назва                                                                                          | Тип      | Unit [Frame] | Range/Enum | Опис                                                                                      |
| ---------------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------- |
| timestamp                                                                                      | `uint64` |                                                                  |            | time since system start (microseconds)                                 |
| geofence_max_dist_triggered     | `bool`   |                                                                  |            | true the check for max distance from Home is triggered                                    |
| geofence_max_alt_triggered      | `bool`   |                                                                  |            | true the check for max altitude above Home is triggered                                   |
| geofence_custom_fence_triggered | `bool`   |                                                                  |            | true the check for custom inclusion/exclusion geofence(s) is triggered |
| geofence_action                                                           | `uint8`  |                                                                  |            | action to take when the geofence is breached                                              |

## Constants

\| Name                                                    | Type    | Value | Description                     |
\| ------------------------------------------------------- | ------- | ----- | ------------------------------- | ------ |
\| <a href="#GF_ACTION_NONE"></a> GF_ACTION_NONE           | `uint8` | 0     | no action on geofence violation |
\| <a href="#GF_ACTION_WARN"></a> GF_ACTION_WARN           | `uint8` | 1     | critical mavlink message        |
\| <a href="#GF_ACTION_LOITER"></a> GF_ACTION_LOITER       | `uint8` | 2     | switch to AUTO                  | LOITER |
\| <a href="#GF_ACTION_RTL"></a> GF_ACTION_RTL             | `uint8` | 3     | switch to AUTO                  | RTL    |
\| <a href="#GF_ACTION_TERMINATE"></a> GF_ACTION_TERMINATE | `uint8` | 4     | flight termination              |
\| <a href="#GF_ACTION_LAND"></a> GF_ACTION_LAND           | `uint8` | 5     | switch to AUTO                  | LAND   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GeofenceResult.msg)

:::details
Click here to see original file

```c
uint64 timestamp                        # time since system start (microseconds)
uint8 GF_ACTION_NONE = 0                # no action on geofence violation
uint8 GF_ACTION_WARN = 1                # critical mavlink message
uint8 GF_ACTION_LOITER = 2              # switch to AUTO|LOITER
uint8 GF_ACTION_RTL = 3                 # switch to AUTO|RTL
uint8 GF_ACTION_TERMINATE = 4           # flight termination
uint8 GF_ACTION_LAND = 5                # switch to AUTO|LAND

bool geofence_max_dist_triggered	# true the check for max distance from Home is triggered
bool geofence_max_alt_triggered		# true the check for max altitude above Home is triggered
bool geofence_custom_fence_triggered	# true the check for custom inclusion/exclusion geofence(s) is triggered

uint8 geofence_action           	# action to take when the geofence is breached
```

:::
