uint64 timestamp # time since system start (microseconds)

float32[2] position_ned # 2-dimensional position setpoint in NED frame [m]
float32 cruising_speed  # (Optional) Specify rover speed (Defaults to maximum speed)

float32 yaw             # [rad] [-pi,pi] from North. Optional, NAN if not set. Mecanum only.
