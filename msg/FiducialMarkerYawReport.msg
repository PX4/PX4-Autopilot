# Relative orientation of landing target in NED (North, East, Down)

uint64 timestamp         # [us] time since system start
uint64 timestamp_sample  # [us] timestamp of the raw observation

float32 yaw_ned          # [rad] [@frame NED] orientation of the target relative to the NED frame [-Pi ; Pi]
float32 yaw_var_ned      # [rad^2] orientation uncertainty
