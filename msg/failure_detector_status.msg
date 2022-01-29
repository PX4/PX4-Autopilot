uint64 timestamp                    # time since system start (microseconds)

# FailureDetector status
bool fd_roll
bool fd_pitch
bool fd_alt
bool fd_ext
bool fd_arm_escs
bool fd_high_wind
bool fd_battery
bool fd_imbalanced_prop

float32 imbalanced_prop_metric      # Metric of the imbalanced propeller check (low-passed)
