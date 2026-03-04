---
pageClass: is-wide-page
---

# OrbitStatus (UORB повідомлення)

ORBIT_YAW_BEHAVIOUR.

**TOPICS:** orbit_status

## Fields

| Назва                              | Тип       | Unit [Frame] | Range/Enum | Опис                                                                                                                                                                                                                                      |
| ---------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                          | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                 |
| radius                             | `float32` |                                                                  |            | Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise. [m]                                             |
| frame                              | `uint8`   |                                                                  |            | The coordinate system of the fields: x, y, z.                                                                                                                                                             |
| x                                  | `float64` |                                                                  |            | X coordinate of center point. Coordinate system depends on frame field: local = x position in meters _ 1e4, global = latitude in degrees _ 1e7. |
| y                                  | `float64` |                                                                  |            | Y coordinate of center point. Coordinate system depends on frame field: local = y position in meters _ 1e4, global = latitude in degrees _ 1e7. |
| z                                  | `float32` |                                                                  |            | Altitude of center point. Coordinate system depends on frame field.                                                                                                                                       |
| yaw_behaviour | `uint8`   |                                                                  |            |                                                                                                                                                                                                                                           |

## Constants

| Назва                                                                                                                                                                                                                                                                | Тип     | Значення | Опис |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER"></a> ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER   | `uint8` | 0        |      |
| <a href="#ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING"></a> ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING                                                           | `uint8` | 1        |      |
| <a href="#ORBIT_YAW_BEHAVIOUR_UNCONTROLLED"></a> ORBIT_YAW_BEHAVIOUR_UNCONTROLLED                                                                                                                     | `uint8` | 2        |      |
| <a href="#ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE"></a> ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE | `uint8` | 3        |      |
| <a href="#ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED"></a> ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED                                                                                              | `uint8` | 4        |      |
| <a href="#ORBIT_YAW_BEHAVIOUR_UNCHANGED"></a> ORBIT_YAW_BEHAVIOUR_UNCHANGED                                                                                                                           | `uint8` | 5        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OrbitStatus.msg)

:::details
Click here to see original file

```c
# ORBIT_YAW_BEHAVIOUR
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1
uint8 ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3
uint8 ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4
uint8 ORBIT_YAW_BEHAVIOUR_UNCHANGED = 5

uint64 timestamp # time since system start (microseconds)
float32 radius   # Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise. [m]
uint8 frame      # The coordinate system of the fields: x, y, z.
float64 x        # X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
float64 y        # Y coordinate of center point. Coordinate system depends on frame field: local = y position in meters * 1e4, global = latitude in degrees * 1e7.
float32 z        # Altitude of center point. Coordinate system depends on frame field.
uint8 yaw_behaviour
```

:::
