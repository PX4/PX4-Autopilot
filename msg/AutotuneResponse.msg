# Synchronous controller inputs/output and the excitation actually consumed.
uint64 timestamp # [us] Controller execution time
uint64 timestamp_sample # [us] Gyroscope sample time
float32 dt # [s] Controller sample interval
float32[3] excitation # [-] Additive normalized torque actually applied
float32[3] torque # [-] Total normalized torque after output filtering
float32[3] angular_velocity # [rad/s] Rate feedback used by the controller
float32[3] angular_acceleration # [rad/s^2] Derivative feedback used by the controller
