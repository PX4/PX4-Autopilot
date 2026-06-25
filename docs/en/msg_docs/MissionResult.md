---
pageClass: is-wide-page
---

# MissionResult (UORB message)

**TOPICS:** mission_result

## Fields

| Name                                                          | Type     | Unit [Frame] | Range/Enum | Description                                                                                         |
| ------------------------------------------------------------- | -------- | ------------ | ---------- | --------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                           | `uint64` |              |            | time since system start (microseconds)                                                              |
| <a id="fld_mission_id"></a>mission_id                         | `uint32` |              |            | Id for the mission for which the result was generated                                               |
| <a id="fld_geofence_id"></a>geofence_id                       | `uint32` |              |            | Id for the corresponding geofence for which the result was generated (used for mission feasibility) |
| <a id="fld_home_position_counter"></a>home_position_counter   | `uint32` |              |            | Counter of the home position for which the result was generated (used for mission feasibility)      |
| <a id="fld_seq_reached"></a>seq_reached                       | `int32`  |              |            | Sequence of the mission item which has been reached, default -1                                     |
| <a id="fld_seq_current"></a>seq_current                       | `uint16` |              |            | Sequence of the current mission item                                                                |
| <a id="fld_seq_total"></a>seq_total                           | `uint16` |              |            | Total number of mission items                                                                       |
| <a id="fld_valid"></a>valid                                   | `bool`   |              |            | true if mission is valid                                                                            |
| <a id="fld_warning"></a>warning                               | `bool`   |              |            | true if mission is valid, but has potentially problematic items leading to safety warnings          |
| <a id="fld_finished"></a>finished                             | `bool`   |              |            | true if mission has been completed                                                                  |
| <a id="fld_failure"></a>failure                               | `bool`   |              |            | true if the mission cannot continue or be completed for some reason                                 |
| <a id="fld_item_do_jump_changed"></a>item_do_jump_changed     | `bool`   |              |            | true if the number of do jumps remaining has changed                                                |
| <a id="fld_item_changed_index"></a>item_changed_index         | `uint16` |              |            | indicate which item has changed                                                                     |
| <a id="fld_item_do_jump_remaining"></a>item_do_jump_remaining | `uint16` |              |            | set to the number of do jumps remaining for that item                                               |
| <a id="fld_execution_mode"></a>execution_mode                 | `uint8`  |              |            | indicates the mode in which the mission is executed                                                 |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MissionResult.msg)

::: details Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint32 mission_id   		# Id for the mission for which the result was generated
uint32 geofence_id  		# Id for the corresponding geofence for which the result was generated (used for mission feasibility)
uint32 home_position_counter  	# Counter of the home position for which the result was generated (used for mission feasibility)

int32 seq_reached		# Sequence of the mission item which has been reached, default -1
uint16 seq_current		# Sequence of the current mission item
uint16 seq_total		# Total number of mission items

bool valid			# true if mission is valid
bool warning			# true if mission is valid, but has potentially problematic items leading to safety warnings
bool finished			# true if mission has been completed
bool failure			# true if the mission cannot continue or be completed for some reason

bool item_do_jump_changed	# true if the number of do jumps remaining has changed
uint16 item_changed_index	# indicate which item has changed
uint16 item_do_jump_remaining	# set to the number of do jumps remaining for that item

uint8 execution_mode	# indicates the mode in which the mission is executed
```

:::
