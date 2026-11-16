# Completion of a temporary motor-direction request.
uint64 timestamp # [us] Time since system start
uint64 timestamp_disarmed # [us] Most recent disarmed actuator_armed sample observed by the driver
uint32 request_id
uint16 supported_motor_mask # Logical motors uniquely mapped to compatible DShot outputs
uint16 motor_mask # Motors addressed by the current request
uint16 reversed_mask # Motors with temporary reversal outstanding
uint8 state
uint8 STATE_IDLE = 0
uint8 STATE_PENDING = 1
uint8 STATE_COMPLETE = 2
uint8 STATE_REJECTED = 3
