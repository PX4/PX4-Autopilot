
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz          # torque setpoint about X, Y, Z body axis (normalized)
float32[3] filtered_xyz # filtered torque setpoint about X, Y, Z body axis (normalized)
float32[3] g1_term      # G1 term of torque setpoint about X, Y, Z body axis (normalized)
float32[3] g2_term      # G2 term of torque setpoint about X, Y, Z body axis (normalized)
float32[3] delta_xyz    # delta torque setpoint about X, Y, Z body axis (normalized)
float32[3] error        # error between filtered torque setpoint and angular acceleration (normalized)

# TOPICS vehicle_torque_setpoint
# TOPICS vehicle_torque_setpoint_virtual_fw vehicle_torque_setpoint_virtual_mc
