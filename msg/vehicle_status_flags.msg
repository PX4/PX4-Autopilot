# define mask for conditions flags
uint16 CONDITION_CALIBRATION_ENABLE_MASK = 1
uint16 CONDITION_SYSTEM_SENSORS_INITIALIZED_MASK = 2
uint16 CONDITION_SYSTEM_PREARM_ERROR_REPORTED_MASK = 4
uint16 CONDITION_SYSTEM_HOTPLUG_TIMEOUT_MASK = 8
uint16 CONDITION_SYSTEM_RETURNED_TO_HOME_MASK = 16
uint16 CONDITION_AUTO_MISSION_AVAILABLE_MASK = 32
uint16 CONDITION_GLOBAL_POSITION_VALID_MASK = 64
uint16 CONDITION_HOME_POSITION_VALID_MASK = 128
uint16 CONDITION_LOCAL_POSITION_VALID_MASK = 256
uint16 CONDITION_LOCAL_ALTITUDE_VALID_MASK = 512
uint16 CONDITION_AIRSPEED_VALID_MASK = 1024
uint16 CONDITION_POWER_INPUT_VALID_MASK = 2048
# uint16 reserved for future implementation
# uint16 reserved for future implementation
# uint16 reserved for future implementation
# uint16 reserved for future implementation

uint16 conditions

# define mask for circuit breaker flags
uint8 CIRCUIT_BREAKER_ENGAGED_POWER_CHECK_MASK = 1
uint8 CIRCUIT_BREAKER_ENGAGED_AIRSPD_CHECK_MASK = 2
uint8 CIRCUIT_BREAKER_ENGAGED_ENGINEFAILURE_CHECK_MASK = 4
uint8 CIRCUIT_BREAKER_ENGAGED_GPSFAILURE_CHECK_MASK = 8
uint8 CIRCUIT_BREAKER_FLIGHT_TERMINATION_DISABLED_MASK = 16
uint8 CIRCUIT_BREAKER_ENGAGED_USB_CHECK_MASK = 32

# 0x0001 circuit_breaker_engaged_power_check
# 0x0002 circuit_breaker_engaged_airspd_check
# 0x0004 circuit_breaker_engaged_enginefailure_check
# 0x0008 circuit_breaker_engaged_gpsfailure_check
# 0x0010 circuit_breaker_flight_termination_disabled
# 0x0020 circuit_breaker_engaged_usb_check

uint8 circuit_breakers

# define mask for other flags
uint16 USB_CONNECTED_MASK = 1
uint16 OFFBOARD_CONTROL_SIGNAL_FOUND_ONCE_MASK = 2
uint16 OFFBOARD_CONTROL_SIGNAL_LOST_MASK = 4
uint16 OFFBOARD_CONTROL_SIGNAL_WEAK_MASK = 8
uint16 OFFBOARD_CONTROL_SET_BY_COMMAND_MASK = 16
uint16 OFFBOARD_CONTROL_LOSS_TIMEOUT_MASK = 32
uint16 RC_SIGNAL_FOUND_ONCE_MASK = 64
uint16 RC_SIGNAL_LOST_CMD_MASK = 128
uint16 RC_INPUT_BLOCKED_MASK = 256
uint16 DATA_LINK_LOST_CMD_MASK = 512
uint16 VTOL_TRANSITION_FAILURE_MASK = 1024
uint16 VTOL_TRANSITION_FAILURE_CMD_MASK = 2048
uint16 GPS_FAILURE_MASK = 4096
uint16 GPS_FAILURE_CMD_MASK = 8192
uint16 BAROMETER_FAILURE_MASK = 16384
uint16 EVER_HAD_BAROMETER_DATA_MASK = 32768

# 0x0001 usb_connected: status of the USB power supply
# 0x0002 offboard_control_signal_found_once
# 0x0004 offboard_control_signal_lost
# 0x0008 offboard_control_signal_weak
# 0x0010 offboard_control_set_by_command: true if the offboard mode was set by a mavlink command and should not be overridden by RC
# 0x0020 offboard_control_loss_timeout: true if offboard is lost for a certain amount of time
# 0x0040 rc_signal_found_once
# 0x0080 rc_signal_lost_cmd: true if RC lost mode is commanded
# 0x0100 rc_input_blocked: set if RC input should be ignored temporarily
# 0x0200 data_link_lost_cmd: datalink to GCS lost mode commanded
# 0x0400 vtol_transition_failure: Set to true if vtol transition failed
# 0x0800 vtol_transition_failure_cmd: Set to true if vtol transition failure mode is commanded
# 0x1000 gps_failure: Set to true if a gps failure is detected
# 0x2000 gps_failure_cmd: Set to true if a gps failure mode is commanded
# 0x4000 barometer_failure: Set to true if a barometer failure is detected
# 0x8000 ever_had_barometer_data: Set to true if ever had valid barometer data before

uint16 other_flags
