uint8 esc_vendor					# Vendor of current ESC
uint32 esc_errorcount					# Number of reported errors by ESC - if supported
int32 esc_rpm						# Motor RPM, negative for reverse rotation [RPM] - if supported
float32 esc_voltage					# Voltage measured from current ESC [V] - if supported
float32 esc_current					# Current measured from current ESC [A] - if supported
float32 esc_temperature					# Temperature measured from current ESC [degC] - if supported
float32 esc_setpoint					# setpoint of current ESC
uint16 esc_setpoint_raw					# setpoint of current ESC (Value sent to ESC)
uint16 esc_address					# Address of current ESC (in most cases 1-8 / must be set by driver)
uint16 esc_version					# Version of current ESC - if supported
uint16 esc_state					# State of ESC - depend on Vendor
