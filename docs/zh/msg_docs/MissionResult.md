---
pageClass: is-wide-page
---

# MissionResult (UORB message)

**TOPICS:** mission_result

## Fields

| 参数名                                                                                   | 类型       | Unit [Frame] | Range/Enum | 描述                                                                                                                     |
| ------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                             | `uint64` |                                                                  |            | time since system start (microseconds)                                                              |
| mission_id                                                       | `uint32` |                                                                  |            | Id for the mission for which the result was generated                                                                  |
| geofence_id                                                      | `uint32` |                                                                  |            | Id for the corresponding geofence for which the result was generated (used for mission feasibility) |
| home_position_counter                       | `uint32` |                                                                  |            | Counter of the home position for which the result was generated (used for mission feasibility)      |
| seq_reached                                                      | `int32`  |                                                                  |            | Sequence of the mission item which has been reached, default -1                                                        |
| seq_current                                                      | `uint16` |                                                                  |            | Sequence of the current mission item                                                                                   |
| seq_total                                                        | `uint16` |                                                                  |            | Total number of mission items                                                                                          |
| valid                                                                                 | `bool`   |                                                                  |            | true if mission is valid                                                                                               |
| warning                                                                               | `bool`   |                                                                  |            | true if mission is valid, but has potentially problematic items leading to safety warnings                             |
| finished                                                                              | `bool`   |                                                                  |            | true if mission has been completed                                                                                     |
| failure                                                                               | `bool`   |                                                                  |            | true if the mission cannot continue or be completed for some reason                                                    |
| item_do_jump_changed   | `bool`   |                                                                  |            | true if the number of do jumps remaining has changed                                                                   |
| item_changed_index                          | `uint16` |                                                                  |            | indicate which item has changed                                                                                        |
| item_do_jump_remaining | `uint16` |                                                                  |            | set to the number of do jumps remaining for that item                                                                  |
| execution_mode                                                   | `uint8`  |                                                                  |            | indicates the mode in which the mission is executed                                                                    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MissionResult.msg)

:::details
Click here to see original file

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
