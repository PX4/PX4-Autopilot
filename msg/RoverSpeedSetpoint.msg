# Rover Velocity Setpoint

uint64 timestamp # [us] Time since system start
float32 speed_body_x # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint in body x direction
float32 speed_body_y # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint in body y direction

# To be deleted
float32 speed # [m/s] [@range -inf (Backwards), inf (Forwards)] Speed setpoint
float32 bearing # [rad] [@range -pi,pi] [@frame NED] [@invalid: NaN, speed is defined in body x direction] Bearing setpoint
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN, Defaults to vehicle yaw] Mecanum only: Yaw setpoint
