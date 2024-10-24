# Position and (optional) heading setpoints with corresponding speed constraints
# Setpoints are intended as inputs to position and heading smoothers, respectively
# Setpoints do not need to be kinematically consistent
# Optional heading setpoints may be specified as controlled by the respective flag
# Unset optional setpoints are not controlled
# Unset optional constraints default to vehicle specifications

uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

# setpoints
float32[3] position # [m] NED local world frame

bool flag_control_heading # true if heading is to be controlled
float32 heading # (optional) [rad] [-pi,pi] from North

# constraints
bool flag_set_max_horizontal_speed # true if setting a non-default horizontal speed limit
float32 max_horizontal_speed # (optional) [m/s] maximum speed (absolute) in the NE-plane

bool flag_set_max_vertical_speed # true if setting a non-default vertical speed limit
float32 max_vertical_speed # (optional) [m/s] maximum speed (absolute) in the D-axis

bool flag_set_max_heading_rate # true if setting a non-default heading rate limit
float32 max_heading_rate # (optional) [rad/s] maximum heading rate (absolute)
