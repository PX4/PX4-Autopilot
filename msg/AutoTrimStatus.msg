# Auto trimming state and current estimate
uint64 timestamp             # Time since system start (microseconds)
uint64 timestamp_sample

float32[3] trim_estimate     # Roll/pitch/yaw trim value obtained from torque setpoints averaged over several seconds
float32[3] trim_estimate_var
float32[3] trim_test         # Same as trim_estimate but done on a shorter period. Used to validate the trim estimate.
float32[3] trim_test_var
float32[3] trim_validated    # Final roll/pitch/yaw trim estimate, verified by trim_test

uint8 STATE_IDLE = 0
uint8 STATE_SAMPLING = 1
uint8 STATE_SAMPLING_TEST = 2
uint8 STATE_VERIFICATION = 3
uint8 STATE_COMPLETE = 4
uint8 STATE_FAIL = 5

uint8 state
