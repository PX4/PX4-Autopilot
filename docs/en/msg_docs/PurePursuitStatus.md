# PurePursuitStatus (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PurePursuitStatus.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 lookahead_distance   # [m] Lookahead distance of pure the pursuit controller
float32 target_bearing       # [rad] Target bearing calculated by the pure pursuit controller
float32 crosstrack_error     # [m] Shortest distance from the vehicle to the path (Positiv: Vehicle is on the right hand side with respect to the oriented path vector, Negativ: Left of the path)
float32 distance_to_waypoint # [m] Distance from the vehicle to the current waypoint
float32 bearing_to_waypoint  # [rad] Bearing towards current waypoint

# TOPICS pure_pursuit_status

```
