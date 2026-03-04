---
pageClass: is-wide-page
---

# FollowTarget (UORB message)

**TOPICS:** follow_target

## Fields

| 명칭                           | 형식        | Unit [Frame] | Range/Enum | 설명                                                        |
| ---------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                    | `uint64`  |                                                                  |            | time since system start (microseconds) |
| lat                          | `float64` |                                                                  |            | target position (deg \* 1e7)           |
| lon                          | `float64` |                                                                  |            | target position (deg \* 1e7)           |
| alt                          | `float32` |                                                                  |            | target position                                           |
| vy                           | `float32` |                                                                  |            | target vel in y                                           |
| vx                           | `float32` |                                                                  |            | target vel in x                                           |
| vz                           | `float32` |                                                                  |            | target vel in z                                           |
| est_cap | `uint8`   |                                                                  |            | target reporting capabilities                             |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FollowTarget.msg)

:::details
Click here to see original file

```c
uint64 timestamp  # time since system start (microseconds)

float64 lat       # target position (deg * 1e7)
float64 lon       # target position (deg * 1e7)
float32 alt       # target position

float32 vy        # target vel in y
float32 vx        # target vel in x
float32 vz        # target vel in z

uint8 est_cap     # target reporting capabilities
```

:::
