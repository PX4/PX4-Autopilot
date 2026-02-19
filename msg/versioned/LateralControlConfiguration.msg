# Fixed Wing Lateral Control Configuration message
#
# Used by the fw_lateral_longitudinal_control module to constrain FixedWingLateralSetpoint messages.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start

float32 lateral_accel_max # [m/s^2] Currently maps to a maximum roll angle, accel_max = tan(roll_max) * GRAVITY
