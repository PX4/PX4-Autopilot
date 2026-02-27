---
pageClass: is-wide-page
---

# RtlStatus (UORB повідомлення)

**TOPICS:** rtl_status

## Fields

| Назва                                                           | Тип      | Unit [Frame] | Range/Enum | Опис                                                                                                                                                                                  |
| --------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                       | `uint64` |                                                                  |            | time since system start (microseconds)                                                                                                                             |
| safe_points_id        | `uint32` |                                                                  |            | unique ID of active set of safe_point_items                                                                                                 |
| is_evaluation_pending | `bool`   |                                                                  |            | flag if the RTL point needs reevaluation (e.g. new safe points available, but need loading).                       |
| has_vtol_approach     | `bool`   |                                                                  |            | flag if approaches are defined for current RTL_TYPE parameter setting                                                                                            |
| rtl_type                                   | `uint8`  |                                                                  |            | Type of RTL chosen                                                                                                                                                                    |
| safe_point_index      | `uint8`  |                                                                  |            | index of the chosen safe point, if in RTL_STATUS_TYPE_DIRECT_SAFE_POINT mode |

## Constants

| Назва                                                                                                                                                                                                  | Тип     | Значення | Опис                                                                                                                                                                                        |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#RTL_STATUS_TYPE_NONE"></a> RTL_STATUS_TYPE_NONE                                                                               | `uint8` | 0        | pending if evaluation can't pe performed currently e.g. when it is still loading the safe points                                                            |
| <a href="#RTL_STATUS_TYPE_DIRECT_SAFE_POINT"></a> RTL_STATUS_TYPE_DIRECT_SAFE_POINT           | `uint8` | 1        | chosen to directly go to a safe point or home position                                                                                                                                      |
| <a href="#RTL_STATUS_TYPE_DIRECT_MISSION_LAND"></a> RTL_STATUS_TYPE_DIRECT_MISSION_LAND       | `uint8` | 2        | going straight to the beginning of the mission landing                                                                                                                                      |
| <a href="#RTL_STATUS_TYPE_FOLLOW_MISSION"></a> RTL_STATUS_TYPE_FOLLOW_MISSION                                      | `uint8` | 3        | Following the mission from start index to mission landing. Start index is current WP if in Mission mode, and closest WP otherwise.                          |
| <a href="#RTL_STATUS_TYPE_FOLLOW_MISSION_REVERSE"></a> RTL_STATUS_TYPE_FOLLOW_MISSION_REVERSE | `uint8` | 4        | Following the mission in reverse from start index to the beginning of the mission. Start index is previous WP if in Mission mode, and closest WP otherwise. |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RtlStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp                      # time since system start (microseconds)

uint32 safe_points_id 		      # unique ID of active set of safe_point_items
bool is_evaluation_pending 	      # flag if the RTL point needs reevaluation (e.g. new safe points available, but need loading).

bool has_vtol_approach 		      # flag if approaches are defined for current RTL_TYPE parameter setting

uint8 rtl_type	      		      # Type of RTL chosen
uint8 safe_point_index 		      # index of the chosen safe point, if in RTL_STATUS_TYPE_DIRECT_SAFE_POINT mode

uint8 RTL_STATUS_TYPE_NONE=0       		# pending if evaluation can't pe performed currently e.g. when it is still loading the safe points
uint8 RTL_STATUS_TYPE_DIRECT_SAFE_POINT=1 	# chosen to directly go to a safe point or home position
uint8 RTL_STATUS_TYPE_DIRECT_MISSION_LAND=2 	# going straight to the beginning of the mission landing
uint8 RTL_STATUS_TYPE_FOLLOW_MISSION=3 		# Following the mission from start index to mission landing. Start index is current WP if in Mission mode, and closest WP otherwise.
uint8 RTL_STATUS_TYPE_FOLLOW_MISSION_REVERSE=4 	# Following the mission in reverse from start index to the beginning of the mission. Start index is previous WP if in Mission mode, and closest WP otherwise.
```

:::
