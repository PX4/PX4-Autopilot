# Rover Steering setpoint

uint32 MESSAGE_VERSION = 0

uint64  timestamp 	          # Time since system start [us]
float32 normalized_steering_angle # Ackermann only: Normalized steering angle [] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not ackermann]
float32 normalized_speed_diff     # Differerntial/Mecanum only: Normalized speed difference between the left and right wheels of the rover [] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not Differential/Mecanum]
