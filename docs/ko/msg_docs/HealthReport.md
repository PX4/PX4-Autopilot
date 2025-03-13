# HealthReport (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/HealthReport.msg)

```c
uint64 timestamp # time since system start (microseconds)

uint64 can_arm_mode_flags              # bitfield for each flight mode (NAVIGATION_STATE_*) if arming is possible
uint64 can_run_mode_flags              # bitfield for each flight mode if it can run

uint64 health_is_present_flags         # flags for each health_component_t
uint64 health_warning_flags
uint64 health_error_flags
# A component is required but missing, if present==0 and error==1

uint64 arming_check_warning_flags
uint64 arming_check_error_flags

```
