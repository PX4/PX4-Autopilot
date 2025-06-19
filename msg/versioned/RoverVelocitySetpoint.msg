# Rover velocity setpoint

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
float32 speed # [m/s] [@range -inf (Backwards), inf (Forwards)] Speed setpoint
float32 bearing # [rad] [@range -pi,pi] [@frame NED] [@invalid: NaN, speed is defined in body x direction] Bearing setpoint
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN, Defaults to vehicle yaw] Mecanum only: Yaw setpoint
