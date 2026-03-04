---
pageClass: is-wide-page
---

# FigureEightStatus (повідомлення UORB)

**TOPICS:** figure_eightstatus

## Fields

| Назва                             | Тип       | Unit [Frame] | Range/Enum | Опис                                                                                                                                                                                                                                      |
| --------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                         | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                 |
| major_radius | `float32` |                                                                  |            | Major axis radius of the figure eight [m]. Positive values orbit clockwise, negative values orbit counter-clockwise.                                  |
| minor_radius | `float32` |                                                                  |            | Minor axis radius of the figure eight [m].                                                                                                                            |
| orientation                       | `float32` |                                                                  |            | Orientation of the major axis of the figure eight [rad].                                                                                                              |
| frame                             | `uint8`   |                                                                  |            | The coordinate system of the fields: x, y, z.                                                                                                                                                             |
| x                                 | `int32`   |                                                                  |            | X coordinate of center point. Coordinate system depends on frame field: local = x position in meters _ 1e4, global = latitude in degrees _ 1e7. |
| y                                 | `int32`   |                                                                  |            | Y coordinate of center point. Coordinate system depends on frame field: local = y position in meters _ 1e4, global = latitude in degrees _ 1e7. |
| z                                 | `float32` |                                                                  |            | Altitude of center point. Coordinate system depends on frame field.                                                                                                                                       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FigureEightStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)
float32 major_radius 	# Major axis radius of the figure eight [m]. Positive values orbit clockwise, negative values orbit counter-clockwise.
float32 minor_radius 	# Minor axis radius of the figure eight [m].
float32 orientation 	# Orientation of the major axis of the figure eight [rad].
uint8 frame      # The coordinate system of the fields: x, y, z.
int32 x          # X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
int32 y        	 # Y coordinate of center point. Coordinate system depends on frame field: local = y position in meters * 1e4, global = latitude in degrees * 1e7.
float32 z        # Altitude of center point. Coordinate system depends on frame field.
```

:::
