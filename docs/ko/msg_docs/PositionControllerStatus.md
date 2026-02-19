---
pageClass: is-wide-page
---

# PositionControllerStatus (UORB message)

**TOPICS:** position_controllerstatus

## Fields

| 명칭                                     | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                                                        |
| -------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------- |
| timestamp                              | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                 |
| nav_roll          | `float32` |                                                                  |            | Roll setpoint [rad]                                                   |
| nav_pitch         | `float32` |                                                                  |            | Pitch setpoint [rad]                                                  |
| nav_bearing       | `float32` |                                                                  |            | Bearing angle[rad]                                                    |
| target_bearing    | `float32` |                                                                  |            | Bearing angle from aircraft to current target [rad]                   |
| xtrack_error      | `float32` |                                                                  |            | Signed track error [m]                                                |
| wp_dist           | `float32` |                                                                  |            | Distance to active (next) waypoint [m]             |
| acceptance_radius | `float32` |                                                                  |            | Current horizontal acceptance radius [m]                              |
| type                                   | `uint8`   |                                                                  |            | Current (applied) position setpoint type (see PositionSetpoint.msg) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PositionControllerStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

float32 nav_roll		# Roll setpoint [rad]
float32 nav_pitch		# Pitch setpoint [rad]
float32 nav_bearing 		# Bearing angle[rad]
float32 target_bearing		# Bearing angle from aircraft to current target [rad]
float32 xtrack_error		# Signed track error [m]
float32 wp_dist			# Distance to active (next) waypoint [m]
float32 acceptance_radius	# Current horizontal acceptance radius [m]
uint8 type			# Current (applied) position setpoint type (see PositionSetpoint.msg)
```

:::
