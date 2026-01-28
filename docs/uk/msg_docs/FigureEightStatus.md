# FigureEightStatus (повідомлення UORB)

[вихідний файл](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FigureEightStatus.msg)

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
