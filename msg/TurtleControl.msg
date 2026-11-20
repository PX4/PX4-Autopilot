# Turtle recovery intent, consumed by control_allocator only in Turtle mode.
uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Manual input sample timestamp
uint32 direction_request_id # Completed reverse request required before applying control
float32 roll # [-1, 1] Manual roll input
float32 pitch # [-1, 1] Manual pitch input
float32 thrust # (0, 1] Maximum normalized motor thrust (TRTL_MAX)
bool enabled
bool driving # Recovery session permits motor thrust
bool ready_to_arm # Mode prerequisites; Commander also applies its normal arming checks
bool disarm_requested # End recovery because input or configuration is no longer valid
