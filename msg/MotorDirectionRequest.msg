# Request temporary reversal for one armed session, without saving ESC configuration.
# The driver restores the whole group on disarm or cancellation.
uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Armed vehicle_status sample authorizing this request
uint32 request_id # Request sequence, echoed by MotorDirectionStatus
uint16 motor_mask # Logical motor indices: Motor 1 is bit 0
