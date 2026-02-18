---
pageClass: is-wide-page
---

# NavigatorMissionItem (UORB message)

**TOPICS:** navigator_missionitem

## Fields

| 参数名                                                            | 类型        | Unit [Frame] | Range/Enum | 描述                                                                                   |
| -------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------ |
| timestamp                                                      | `uint64`  |                                                                  |            | time since system start (microseconds)                            |
| sequence_current                          | `uint16`  |                                                                  |            | Sequence of the current mission item                                                 |
| nav_cmd                                   | `uint16`  |                                                                  |            |                                                                                      |
| latitude                                                       | `float32` |                                                                  |            |                                                                                      |
| longitude                                                      | `float32` |                                                                  |            |                                                                                      |
| time_inside                               | `float32` |                                                                  |            | time that the MAV should stay inside the radius before advancing in seconds          |
| acceptance_radius                         | `float32` |                                                                  |            | default radius in which the mission is accepted as reached in meters                 |
| loiter_radius                             | `float32` |                                                                  |            | loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise       |
| yaw                                                            | `float32` |                                                                  |            | in radians NED -PI..+PI, NAN means don't change yaw  |
| altitude                                                       | `float32` |                                                                  |            | altitude in meters (AMSL)                                         |
| frame                                                          | `uint8`   |                                                                  |            | mission frame                                                                        |
| origin                                                         | `uint8`   |                                                                  |            | mission item origin (onboard or mavlink)                          |
| loiter_exit_xtrack   | `bool`    |                                                                  |            | exit xtrack location: 0 for center of loiter wp, 1 for exit location |
| force_heading                             | `bool`    |                                                                  |            | heading needs to be reached                                                          |
| altitude_is_relative | `bool`    |                                                                  |            | true if altitude is relative from start point                                        |
| autocontinue                                                   | `bool`    |                                                                  |            | true if next waypoint should follow after this one                                   |
| vtol_back_transition | `bool`    |                                                                  |            | part of the vtol back transition sequence                                            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/NavigatorMissionItem.msg)

:::details
Click here to see original file

```c
uint64 timestamp                 # time since system start (microseconds)

uint16 sequence_current          # Sequence of the current mission item

uint16 nav_cmd

float32 latitude
float32 longitude

float32 time_inside              # time that the MAV should stay inside the radius before advancing in seconds
float32 acceptance_radius        # default radius in which the mission is accepted as reached in meters
float32 loiter_radius            # loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise
float32 yaw                      # in radians NED -PI..+PI, NAN means don't change yaw
float32 altitude                 # altitude in meters (AMSL)

uint8 frame                      # mission frame
uint8 origin                     # mission item origin (onboard or mavlink)

bool loiter_exit_xtrack          # exit xtrack location: 0 for center of loiter wp, 1 for exit location
bool force_heading               # heading needs to be reached
bool altitude_is_relative        # true if altitude is relative from start point
bool autocontinue                # true if next waypoint should follow after this one
bool vtol_back_transition        # part of the vtol back transition sequence
```

:::
