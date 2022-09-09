uint64 timestamp		# time since system start (microseconds)

uint16 free_memory_bytes

float32 voltage_v		# Servo rail voltage in volts
float32 rssi_v			# RSSI pin voltage in volts

# PX4IO status flags (PX4IO_P_STATUS_FLAGS)
bool status_fmu_ok
bool status_raw_pwm

bool status_rc_ok
bool status_rc_dsm
bool status_rc_ppm
bool status_rc_sbus
bool status_rc_st24
bool status_rc_sumd

bool status_safety_button_event # px4io safety button was pressed for longer than 1 second

# PX4IO alarms (PX4IO_P_STATUS_ALARMS)
bool alarm_pwm_error
bool alarm_rc_lost

# PX4IO arming (PX4IO_P_SETUP_ARMING)
bool arming_fmu_armed
bool arming_io_arm_ok

uint16[8] pwm

uint16[4] pwm_group_rate_hz

uint16[18] raw_inputs
