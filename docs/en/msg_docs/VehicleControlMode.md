# VehicleControlMode (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleControlMode.msg)

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp		# time since system start (microseconds)
bool flag_armed			# synonym for actuator_armed.armed

bool flag_multicopter_position_control_enabled

bool flag_control_manual_enabled		# true if manual input is mixed in
bool flag_control_auto_enabled			# true if onboard autopilot should act
bool flag_control_offboard_enabled		# true if offboard control should be used
bool flag_control_position_enabled		# true if position is controlled
bool flag_control_velocity_enabled		# true if horizontal velocity (implies direction) is controlled
bool flag_control_altitude_enabled		# true if altitude is controlled
bool flag_control_climb_rate_enabled		# true if climb rate is controlled
bool flag_control_acceleration_enabled		# true if acceleration is controlled
bool flag_control_attitude_enabled		# true if attitude stabilization is mixed in
bool flag_control_rates_enabled			# true if rates are stabilized
bool flag_control_allocation_enabled		# true if control allocation is enabled
bool flag_control_termination_enabled		# true if flighttermination is enabled

# TODO: use dedicated topic for external requests
uint8 source_id                  # Mode ID (nav_state)

# TOPICS vehicle_control_mode config_control_setpoints

```
