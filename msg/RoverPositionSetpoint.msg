uint64 timestamp # time since system start (microseconds)

float32[2] position_ned # 2 Dimensional position setpoint in NED frame
float32 cruising_speed  # (Optional) Specify rover speed (Defaults to maximum speed)

float32 yaw             # (Optional) Specify rover yaw (Mecanum only)

# TOPICS rover_position_setpoint
