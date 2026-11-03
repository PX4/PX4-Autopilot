# Defines which controllers should run

uint64 timestamp # time since system start (microseconds)
bool flag_armed	# synonym for actuator_armed.armed

bool flag_multicopter_position_control_enabled

bool flag_control_manual_enabled # true if manual input is mixed in
bool flag_control_auto_enabled # true if onboard autopilot should act
bool flag_control_offboard_enabled # true if offboard control should be used
bool flag_control_position_enabled # true if position is controlled
bool flag_control_velocity_enabled # true if horizontal velocity (implies direction) is controlled
bool flag_control_altitude_enabled # true if altitude is controlled
bool flag_control_climb_rate_enabled # true if climb rate is controlled
bool flag_control_attitude_enabled # true if attitude stabilization is mixed in
bool flag_control_rates_enabled	 # true if rates are stabilized
bool flag_control_allocation_enabled # true if control allocation is enabled

uint8 controllable_axes # Bitmask of CONTROLLABLE_AXIS_* the vehicle can currently control, derived from the control allocation effectiveness. A cleared bit (e.g. yaw after a motor failure) tells consumers not to wait for that axis.
uint8 CONTROLLABLE_AXIS_ROLL = 1
uint8 CONTROLLABLE_AXIS_PITCH = 2
uint8 CONTROLLABLE_AXIS_YAW = 4
uint8 CONTROLLABLE_AXIS_THRUST_X = 8
uint8 CONTROLLABLE_AXIS_THRUST_Y = 16
uint8 CONTROLLABLE_AXIS_THRUST_Z = 32
uint8 CONTROLLABLE_AXIS_ALL = 63
