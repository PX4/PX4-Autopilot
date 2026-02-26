---
pageClass: is-wide-page
---

# PurePursuitStatus (UORB message)

Pure pursuit status.

**TOPICS:** pure_pursuitstatus

## Fields

| 参数名                                                            | 类型        | Unit [Frame] | Range/Enum                                                                                                                                                    | 描述                                                       |
| -------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------- |
| timestamp                                                      | `uint64`  | us                                                               |                                                                                                                                                               | Time since system start                                  |
| lookahead_distance                        | `float32` | 米                                                                | [0 : inf]                                                                                 | Lookahead distance of pure the pursuit controller        |
| target_bearing                            | `float32` | rad [NED]    | [-pi : pi]                                                                                | Target bearing calculated by the pure pursuit controller |
| crosstrack_error                          | `float32` | 米                                                                | [-inf (Left of the path) : inf (Right of the path)] | Shortest distance from the vehicle to the path           |
| distance_to_waypoint | `float32` | 米                                                                | [-inf : inf]                                                                              | Distance from the vehicle to the current waypoint        |
| bearing_to_waypoint  | `float32` | rad [NED]    | [-pi : pi]                                                                                | Bearing towards current waypoint                         |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PurePursuitStatus.msg)

:::details
Click here to see original file

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
