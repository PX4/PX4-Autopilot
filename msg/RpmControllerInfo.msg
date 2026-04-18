# Closed-loop RPM controller diagnostics published by the DShot driver.
# Only populated when bidirectional DShot is enabled (DSHOT_BIDIR_EN).
# The RPM PID runs per motor and adjusts the DShot command to track a target RPM.

# TOPICS rpm_controller_info

uint64 timestamp             # [us] time since system boot

uint8 num_motors             # number of valid entries in the arrays

# Per-motor RPM control info (index 0..num_motors-1)
float32[16] rpm_setpoint     # [rpm] desired RPM derived from mixer output
float32[16] rpm_measured     # [rpm] measured RPM from bidirectional DShot
float32[16] rpm_error        # [rpm] setpoint - measured

# Control command output
float32[16] cmd_norm         # [-] normalized command after PID correction (0..1)
uint16[16] dshot_command     # [DShot units] final DShot throttle value sent to ESC
