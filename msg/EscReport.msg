uint64 timestamp # [us] Time since system start

uint32 esc_errorcount # [-] Number of reported errors by ESC - if supported
uint8 esc_errorcount_type # [@enum ERRORCOUNT_TYPE] What esc_errorcount counts, ESC vendors differ

uint8 ERRORCOUNT_TYPE_UNKNOWN = 0 # Not reported or meaning unknown, esc_errorcount must not be evaluated
uint8 ERRORCOUNT_TYPE_ESC_FAULTS = 1 # Faults counted by the ESC internally, cumulative
uint8 ERRORCOUNT_TYPE_TELEMETRY_ERRORS = 2 # Telemetry decode errors counted by the autopilot, several sources combined, cumulative
uint8 ERRORCOUNT_TYPE_SERIAL_TELEMETRY_ERRORS = 3 # Serial (UART) telemetry decode errors counted by the autopilot, cumulative
uint8 ERRORCOUNT_TYPE_BDSHOT_TELEMETRY_ERRORS = 4 # Bidirectional DShot telemetry decode errors counted by the autopilot, cumulative
uint8 ERRORCOUNT_TYPE_CAN_TEC = 5 # Live CAN transmit error counter
uint8 ERRORCOUNT_TYPE_CAN_REC = 6 # Live CAN receive error counter
uint8 ERRORCOUNT_TYPE_CAN_TEC_REC_MAX = 7 # Larger of the live CAN transmit and receive error counters
uint8 ERRORCOUNT_TYPE_CAN_TEC_REC_PACKED = 8 # Both live CAN error counters: TEC in the upper 16 bit, REC in the lower 16 bit
uint8 ERRORCOUNT_TYPE_CAN_ERRORS = 9 # CAN transmit and receive errors accumulated since the ESC powered on

int32 esc_rpm # [rpm] Motor RPM, negative for reverse rotation - if supported
float32 esc_voltage # [V] Voltage measured from current ESC - if supported
float32 esc_current # [A] Current measured from current ESC - if supported
float32 esc_temperature # [degC] Temperature measured from current ESC - if supported
int16 motor_temperature # [degC] Temperature measured from current motor - if supported

uint8 esc_state # [-] State of ESC - depend on Vendor

uint8 actuator_function # [-] Actuator output function (one of Motor1...MotorN)

uint8 ACTUATOR_FUNCTION_MOTOR1 = 101
uint8 ACTUATOR_FUNCTION_MOTOR_MAX = 112 # output_functions.yaml Motor.start + Motor.count - 1

uint16 failures # [@enum FAILURE] Bitmask to indicate the internal ESC faults

uint8 FAILURE_OVER_CURRENT = 0 # (1 << 0)
uint8 FAILURE_OVER_VOLTAGE = 1 # (1 << 1)
uint8 FAILURE_MOTOR_OVER_TEMPERATURE = 2 # (1 << 2)
uint8 FAILURE_OVER_RPM = 3 # (1 << 3)
uint8 FAILURE_INCONSISTENT_CMD = 4 # (1 << 4) Set if ESC received an inconsistent command (i.e out of boundaries)
uint8 FAILURE_MOTOR_STUCK = 5 # (1 << 5)
uint8 FAILURE_GENERIC = 6 # (1 << 6)
uint8 FAILURE_MOTOR_WARN_TEMPERATURE = 7 # (1 << 7)
uint8 FAILURE_WARN_ESC_TEMPERATURE = 8 # (1 << 8)
uint8 FAILURE_OVER_ESC_TEMPERATURE = 9 # (1 << 9)
uint8 ESC_FAILURE_COUNT = 10 # Counter - keep it as last element!

int8 esc_power # [%] [@range 0,100] Applied power (negative values reserved)
