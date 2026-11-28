---
pageClass: is-wide-page
---

# GeofenceResult (UORB message)

**TOPICS:** geofence_result

## Fields

| Name                                                                                      | Type     | Unit [Frame] | Range/Enum | Description                                                                                    |
| ----------------------------------------------------------------------------------------- | -------- | ------------ | ---------- | ---------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                       | `uint64` |              |            | time since system start (microseconds)                                                         |
| <a id="fld_geofence_emer_max_dist_triggered"></a>geofence_emer_max_dist_triggered         | `bool`   |              |            | true if the emergency geofence check for max distance from Home is triggered                   |
| <a id="fld_geofence_emer_max_alt_triggered"></a>geofence_emer_max_alt_triggered           | `bool`   |              |            | true if the emergency geofence check for max altitude above Home is triggered                  |
| <a id="fld_geofence_emer_custom_fence_triggered"></a>geofence_emer_custom_fence_triggered | `bool`   |              |            | true if the emergency geofence check for custom inclusion/exclusion geofence(s) is triggered   |
| <a id="fld_geofence_cont_max_dist_triggered"></a>geofence_cont_max_dist_triggered         | `bool`   |              |            | true if the contingency geofence check for max distance from Home is triggered                 |
| <a id="fld_geofence_cont_custom_fence_triggered"></a>geofence_cont_custom_fence_triggered | `bool`   |              |            | true if the contingency geofence check for custom inclusion/exclusion geofence(s) is triggered |
| <a id="fld_geofence_emer_action"></a>geofence_emer_action                                 | `uint8`  |              |            | action to take when the emergency geofence is breached                                         |
| <a id="fld_geofence_cont_action"></a>geofence_cont_action                                 | `uint8`  |              |            | action to take when the contingency geofence is breached, GF_ACTION_NONE while it is disabled  |

## Constants

| Name                                                  | Type    | Value | Description                     |
| ----------------------------------------------------- | ------- | ----- | ------------------------------- | ------ |
| <a id="#GF_ACTION_NONE"></a> GF_ACTION_NONE           | `uint8` | 0     | no action on geofence violation |
| <a id="#GF_ACTION_WARN"></a> GF_ACTION_WARN           | `uint8` | 1     | critical mavlink message        |
| <a id="#GF_ACTION_LOITER"></a> GF_ACTION_LOITER       | `uint8` | 2     | switch to AUTO                  | LOITER |
| <a id="#GF_ACTION_RTL"></a> GF_ACTION_RTL             | `uint8` | 3     | switch to AUTO                  | RTL    |
| <a id="#GF_ACTION_TERMINATE"></a> GF_ACTION_TERMINATE | `uint8` | 4     | flight termination              |
| <a id="#GF_ACTION_LAND"></a> GF_ACTION_LAND           | `uint8` | 5     | switch to AUTO                  | LAND   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GeofenceResult.msg)

::: details Click here to see original file

```c
uint64 timestamp                        # time since system start (microseconds)
uint8 GF_ACTION_NONE = 0                # no action on geofence violation
uint8 GF_ACTION_WARN = 1                # critical mavlink message
uint8 GF_ACTION_LOITER = 2              # switch to AUTO|LOITER
uint8 GF_ACTION_RTL = 3                 # switch to AUTO|RTL
uint8 GF_ACTION_TERMINATE = 4           # flight termination
uint8 GF_ACTION_LAND = 5                # switch to AUTO|LAND

bool geofence_emer_max_dist_triggered	# true if the emergency geofence check for max distance from Home is triggered
bool geofence_emer_max_alt_triggered	# true if the emergency geofence check for max altitude above Home is triggered
bool geofence_emer_custom_fence_triggered	# true if the emergency geofence check for custom inclusion/exclusion geofence(s) is triggered
bool geofence_cont_max_dist_triggered	# true if the contingency geofence check for max distance from Home is triggered
bool geofence_cont_custom_fence_triggered	# true if the contingency geofence check for custom inclusion/exclusion geofence(s) is triggered

uint8 geofence_emer_action       	# action to take when the emergency geofence is breached
uint8 geofence_cont_action       	# action to take when the contingency geofence is breached, GF_ACTION_NONE while it is disabled
```

:::
