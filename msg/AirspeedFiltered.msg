# Filtered airspeed for torque scaling
#
# This is the low-pass filtered calibrated airspeed used for control surface torque scaling
# Published by fw_rate_control for debugging purposes

uint64 timestamp                       # [us] Time since system start
float32 filtered_airspeed_m_s          # [m/s] Filtered calibrated airspeed
float32 unfiltered_airspeed_m_s        # [m/s] Raw calibrated airspeed (for comparison)
