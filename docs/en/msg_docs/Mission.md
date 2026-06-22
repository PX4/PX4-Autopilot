---
pageClass: is-wide-page
---

# Mission (UORB message)

**TOPICS:** mission

## Fields

| Name                                                      | Type     | Unit [Frame] | Range/Enum | Description                                                                         |
| --------------------------------------------------------- | -------- | ------------ | ---------- | ----------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                       | `uint64` |              |            | time since system start (microseconds)                                              |
| <a id="fld_mission_dataman_id"></a>mission_dataman_id     | `uint8`  |              |            | default 0, there are two offboard storage places in the dataman: 0 or 1             |
| <a id="fld_fence_dataman_id"></a>fence_dataman_id         | `uint8`  |              |            | default 0, there are two offboard storage places in the dataman: 0 or 1             |
| <a id="fld_safepoint_dataman_id"></a>safepoint_dataman_id | `uint8`  |              |            | default 0, there are two offboard storage places in the dataman: 0 or 1             |
| <a id="fld_count"></a>count                               | `uint16` |              |            | count of the missions stored in the dataman                                         |
| <a id="fld_current_seq"></a>current_seq                   | `int32`  |              |            | default -1, start at the one changed latest                                         |
| <a id="fld_land_start_index"></a>land_start_index         | `int32`  |              |            | Index of the land start marker, if unavailable index of the land item, -1 otherwise |
| <a id="fld_land_index"></a>land_index                     | `int32`  |              |            | Index of the land item, -1 otherwise                                                |
| <a id="fld_mission_id"></a>mission_id                     | `uint32` |              |            | indicates updates to the mission, reload from dataman if changed                    |
| <a id="fld_geofence_id"></a>geofence_id                   | `uint32` |              |            | indicates updates to the geofence, reload from dataman if changed                   |
| <a id="fld_safe_points_id"></a>safe_points_id             | `uint32` |              |            | indicates updates to the safe points, reload from dataman if changed                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Mission.msg)

::: details Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)
uint8 mission_dataman_id	# default 0, there are two offboard storage places in the dataman: 0 or 1
uint8 fence_dataman_id		# default 0, there are two offboard storage places in the dataman: 0 or 1
uint8 safepoint_dataman_id	# default 0, there are two offboard storage places in the dataman: 0 or 1

uint16 count		# count of the missions stored in the dataman
int32 current_seq	# default -1, start at the one changed latest

int32 land_start_index  # Index of the land start marker, if unavailable index of the land item, -1 otherwise
int32 land_index 	# Index of the land item, -1 otherwise

uint32 mission_id # indicates updates to the mission, reload from dataman if changed
uint32 geofence_id # indicates updates to the geofence, reload from dataman if changed
uint32 safe_points_id # indicates updates to the safe points, reload from dataman if changed
```

:::
