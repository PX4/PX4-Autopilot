---
pageClass: is-wide-page
---

# PurePursuitStatus (UORB message)

Pure pursuit status.

**TOPICS:** pure_pursuit_status

## Fields

| Name                                                      | Type      | Unit [Frame] | Range/Enum                                          | Description                                              |
| --------------------------------------------------------- | --------- | ------------ | --------------------------------------------------- | -------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                       | `uint64`  | us           |                                                     | Time since system start                                  |
| <a id="fld_lookahead_distance"></a>lookahead_distance     | `float32` | m            | [0 : inf]                                           | Lookahead distance of pure the pursuit controller        |
| <a id="fld_target_bearing"></a>target_bearing             | `float32` | rad [NED]    | [-pi : pi]                                          | Target bearing calculated by the pure pursuit controller |
| <a id="fld_crosstrack_error"></a>crosstrack_error         | `float32` | m            | [-inf (Left of the path) : inf (Right of the path)] | Shortest distance from the vehicle to the path           |
| <a id="fld_distance_to_waypoint"></a>distance_to_waypoint | `float32` | m            | [-inf : inf]                                        | Distance from the vehicle to the current waypoint        |
| <a id="fld_bearing_to_waypoint"></a>bearing_to_waypoint   | `float32` | rad [NED]    | [-pi : pi]                                          | Bearing towards current waypoint                         |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PurePursuitStatus.msg)

::: details Click here to see original file

```c
# Pure pursuit status

uint64 timestamp              # [us] Time since system start
float32 lookahead_distance    # [m] [@range 0, inf] Lookahead distance of pure the pursuit controller
float32 target_bearing        # [rad] [@range -pi, pi] [@frame NED] Target bearing calculated by the pure pursuit controller
float32 crosstrack_error      # [m] [@range -inf (Left of the path), inf (Right of the path)] Shortest distance from the vehicle to the path
float32 distance_to_waypoint  # [m] [@range -inf, inf]Distance from the vehicle to the current waypoint
float32 bearing_to_waypoint   # [rad] [@range -pi, pi] [@frame NED]Bearing towards current waypoint
```

:::
