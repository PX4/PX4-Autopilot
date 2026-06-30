---
pageClass: is-wide-page
---

# NavigatorMissionItem (UORB message)

**TOPICS:** navigator_mission_item

## Fields

| Name                                                      | Type      | Unit [Frame] | Range/Enum | Description                                                                    |
| --------------------------------------------------------- | --------- | ------------ | ---------- | ------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                       | `uint64`  |              |            | time since system start (microseconds)                                         |
| <a id="fld_sequence_current"></a>sequence_current         | `uint16`  |              |            | Sequence of the current mission item                                           |
| <a id="fld_nav_cmd"></a>nav_cmd                           | `uint16`  |              |            |
| <a id="fld_latitude"></a>latitude                         | `float32` |              |            |
| <a id="fld_longitude"></a>longitude                       | `float32` |              |            |
| <a id="fld_time_inside"></a>time_inside                   | `float32` |              |            | time that the MAV should stay inside the radius before advancing in seconds    |
| <a id="fld_acceptance_radius"></a>acceptance_radius       | `float32` |              |            | default radius in which the mission is accepted as reached in meters           |
| <a id="fld_loiter_radius"></a>loiter_radius               | `float32` |              |            | loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise |
| <a id="fld_yaw"></a>yaw                                   | `float32` |              |            | in radians NED -PI..+PI, NAN means don't change yaw                            |
| <a id="fld_altitude"></a>altitude                         | `float32` |              |            | altitude in meters (AMSL)                                                      |
| <a id="fld_frame"></a>frame                               | `uint8`   |              |            | mission frame                                                                  |
| <a id="fld_origin"></a>origin                             | `uint8`   |              |            | mission item origin (onboard or mavlink)                                       |
| <a id="fld_loiter_exit_xtrack"></a>loiter_exit_xtrack     | `bool`    |              |            | exit xtrack location: 0 for center of loiter wp, 1 for exit location           |
| <a id="fld_force_heading"></a>force_heading               | `bool`    |              |            | heading needs to be reached                                                    |
| <a id="fld_altitude_is_relative"></a>altitude_is_relative | `bool`    |              |            | true if altitude is relative from start point                                  |
| <a id="fld_autocontinue"></a>autocontinue                 | `bool`    |              |            | true if next waypoint should follow after this one                             |
| <a id="fld_vtol_back_transition"></a>vtol_back_transition | `bool`    |              |            | part of the vtol back transition sequence                                      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/NavigatorMissionItem.msg)

::: details Click here to see original file

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
