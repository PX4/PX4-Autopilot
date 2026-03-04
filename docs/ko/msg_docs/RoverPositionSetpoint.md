---
pageClass: is-wide-page
---

# RoverPositionSetpoint (UORB message)

Rover Position Setpoint.

**TOPICS:** rover_positionsetpoint

## Fields

| 명칭                                  | 형식           | Unit [Frame] | Range/Enum                                                                       | 설명                                                                                                                                          |
| ----------------------------------- | ------------ | ---------------------------------------------------------------- | -------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                           | `uint64`     | us                                                               |                                                                                  | Time since system start                                                                                                                     |
| position_ned   | `float32[2]` | m [NED]      | [-inf : inf] | Target position                                                                                                                             |
| start_ned      | `float32[2]` | m [NED]      | [-inf : inf] | Start position which specifies a line for the rover to track (Invalid: NaN Defaults to vehicle position) |
| cruising_speed | `float32`    | m/s                                                              | [0 : inf]    | Cruising speed (Invalid: NaN Defaults to maximum speed)                                                  |
| arrival_speed  | `float32`    | m/s                                                              | [0 : inf]    | Speed the rover should arrive at the target with (Invalid: NaN Defaults to 0)                            |
| yaw                                 | `float32`    | rad [NED]    | [-pi : pi]   | Mecanum only: Specify vehicle yaw during travel (Invalid: NaN Defaults to vehicle yaw)   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverPositionSetpoint.msg)

:::details
Click here to see original file

```c
# Rover Position Setpoint

uint64 timestamp         # [us] Time since system start
float32[2] position_ned  # [m] [@range -inf, inf] [@frame NED] Target position
float32[2] start_ned     # [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to vehicle position] Start position which specifies a line for the rover to track
float32 cruising_speed   # [m/s] [@range 0, inf] [@invalid NaN Defaults to maximum speed] Cruising speed
float32 arrival_speed    # [m/s] [@range 0, inf] [@invalid NaN Defaults to 0] Speed the rover should arrive at the target with
float32 yaw              # [rad] [@range -pi,pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw] Mecanum only: Specify vehicle yaw during travel
```

:::
