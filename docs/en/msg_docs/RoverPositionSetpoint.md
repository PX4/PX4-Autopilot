# RoverPositionSetpoint (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverPositionSetpoint.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32[2] position_ned # 2-dimensional position setpoint in NED frame [m]
float32[2] start_ned	# (Optional) 2-dimensional start position in NED frame used to define the line that the rover will track to position_ned [m] (Defaults to vehicle position)
float32 cruising_speed  # (Optional) Specify rover speed [m/s] (Defaults to maximum speed)
float32 arrival_speed   # (Optional) Specify arrival speed [m/s] (Defaults to zero)

float32 yaw             # [rad] [-pi,pi] from North. Optional, NAN if not set. Mecanum only. (Defaults to vehicle yaw)

```
