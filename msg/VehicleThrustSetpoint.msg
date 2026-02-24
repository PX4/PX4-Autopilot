# Vehicle thrust setpoint
#
# This is the thrust setpoint provided by the controller and fed into the control allocator.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the data sample on which this message is based

float32[3] xyz          # [-] [@range -1, 1] Thrust setpoint along X, Y, Z body axis. If set to NAN the motors affecting this axis are stopped.

# TOPICS vehicle_thrust_setpoint
# TOPICS vehicle_thrust_setpoint_virtual_fw vehicle_thrust_setpoint_virtual_mc
