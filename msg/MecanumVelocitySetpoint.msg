uint64 timestamp # time since system start (microseconds)

float32[2] velocity_ned # 2 Dimensional velocity setpoint in NED frame
float32 yaw 		# (Optional, defaults to current vehicle yaw) Vehicle yaw setpoint in NED frame [rad]

float32[2] velocity_xyz # (Exclusive with velocity_ned) 2 Dimensional velocity setpoint in body X, Y, Z frame (X: Positiv = forwards, Negativ = backwards, Y: Positiv: right, Negativ: left)
