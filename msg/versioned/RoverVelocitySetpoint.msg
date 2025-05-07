# Rover Velocity Setpoint

uint32 MESSAGE_VERSION = 0

uint64  timestamp # Time since system start [us]
float32 speed     # Speed setpoint [m/s] [@range -inf (Backwards), inf (Forwards)]
float32 bearing   # Bearing setpoint [rad] [@range -pi,pi] [@frame NED]
float32 yaw 	  # Mecanum only: Yaw setpoint [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw]
