uint64 timestamp                    # time since system start (microseconds)

# FailureDetector status
bool fd_roll
bool fd_pitch
bool fd_alt
bool fd_ext
bool fd_arm_escs
bool fd_battery
bool fd_imbalanced_prop
bool fd_motor
bool fd_impact                      # Impact detected (acceleration spike), latched until disarm
bool fd_crash                       # Crash detected (impact followed by no movement while not landed), latched until disarm

float32 imbalanced_prop_metric      # Metric of the imbalanced propeller check (low-passed)
float32 impact_metric               # Peak of the impact metric (short-window averaged specific force norm) since last publication (m/s^2)
uint16 motor_failure_mask           # Bit-mask with motor indices, indicating critical motor failures
uint16 motor_stop_mask              # Bitmaks of motors stopped by failure injection
