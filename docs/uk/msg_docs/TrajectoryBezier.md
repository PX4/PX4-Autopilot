# TrjectoryBezier (UORB повідомлення)

Опис траекторії Безьє. Див. також Повідомлення Mavlink TRAJECTORY
Тема trajectory_bezier описує кожну точку маршруту, визначену в vehicle_trajectory_bezier

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TrajectoryBezier.msg)

```c
# Bezier Trajectory description. See also Mavlink TRAJECTORY msg
# The topic trajectory_bezier describe each waypoint defined in vehicle_trajectory_bezier

uint64 timestamp		# time since system start (microseconds)

float32[3] position     # local position x,y,z (metres)
float32 yaw             # yaw angle (rad)
float32 delta           # time it should take to get to this waypoint, if this is the final waypoint (seconds)

```
