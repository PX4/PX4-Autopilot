
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz          # torque setpoint about X, Y, Z body axis (normalized)
float32[3] g1_term      # G1 term of torque setpoint about X, Y, Z body axis (normalized)
float32[3] g2_term      # G2 term of torque setpoint about X, Y, Z body axis (normalized)
float32[3] pid_xyz      # PID torque setpoint about X, Y, Z body axis (normalized)
float32[3] indi_torque_xyz # INDI torque setpoint about X, Y, Z body axis (normalized)
float32[3] measured_body_torque_xyz # measured body torque about X, Y, Z body axis (normalized)

# TOPICS vehicle_torque_setpoint
# TOPICS vehicle_torque_setpoint_virtual_fw vehicle_torque_setpoint_virtual_mc
