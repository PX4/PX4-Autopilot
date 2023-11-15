# Position and (optional) heading setpoints with corresponding speed constraints
# Setpoints are intended as inputs to position and heading smoothers, respectively
# Setpoints do not need to be kinematically consistent
# Optional heading setpoints may be specified as controlled by the respective flag
# Unset optional setpoints are not controlled
# Unset optional constraints default to vehicle specifications

uint64 timestamp # time since system start (microseconds)

# setpoints
float32[3] position # [m] NED local world frame
float32 heading # (optional) [rad] [-pi,pi] from North

# setpoint flags
bool flag_control_heading # true if heading is to be controlled

# constraints
float32 max_horizontal_speed # (optional) [m/s] maximum speed (absolute) in the NE-plane
float32 max_vertical_speed # (optional) [m/s] maximum speed (absolute) in the D-axis
float32 max_heading_rate # (optional) [rad/s] maximum heading rate (absolute)

# constraint flags
bool flag_set_max_horizontal_speed # true if setting a non-default horizontal speed limit
bool flag_set_max_vertical_speed # true if setting a non-default vertical speed limit
bool flag_set_max_heading_rate # true if setting a non-default heading rate limit
