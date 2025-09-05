# Surface Vehicle Steering setpoint

uint64 timestamp # [us] Time since system start
float32 normalized_steering_setpoint # [@range -1 (Left), 1 (Right)] [@frame Body] Ackermann: Normalized steering angle, Differential/Omnidirectional: Normalized speed difference between the left and right wheels
