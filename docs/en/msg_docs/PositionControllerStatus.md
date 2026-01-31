# PositionControllerStatus (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PositionControllerStatus.msg)

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
