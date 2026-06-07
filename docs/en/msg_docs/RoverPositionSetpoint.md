---
pageClass: is-wide-page
---

# RoverPositionSetpoint (UORB message)

Rover Position Setpoint.

**TOPICS:** rover_position_setpoint

## Fields

| Name                                          | Type         | Unit [Frame] | Range/Enum   | Description                                                                                              |
| --------------------------------------------- | ------------ | ------------ | ------------ | -------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp           | `uint64`     | us           |              | Time since system start                                                                                  |
| <a id="fld_position_ned"></a>position_ned     | `float32[2]` | m [NED]      | [-inf : inf] | Target position                                                                                          |
| <a id="fld_start_ned"></a>start_ned           | `float32[2]` | m [NED]      | [-inf : inf] | Start position which specifies a line for the rover to track (Invalid: NaN Defaults to vehicle position) |
| <a id="fld_cruising_speed"></a>cruising_speed | `float32`    | m/s          | [0 : inf]    | Cruising speed (Invalid: NaN Defaults to maximum speed)                                                  |
| <a id="fld_arrival_speed"></a>arrival_speed   | `float32`    | m/s          | [0 : inf]    | Speed the rover should arrive at the target with (Invalid: NaN Defaults to 0)                            |
| <a id="fld_yaw"></a>yaw                       | `float32`    | rad [NED]    | [-pi : pi]   | Mecanum only: Specify vehicle yaw during travel (Invalid: NaN Defaults to vehicle yaw)                   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverPositionSetpoint.msg)

::: details Click here to see original file

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
