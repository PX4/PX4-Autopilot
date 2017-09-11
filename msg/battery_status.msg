float32 voltage_v		# Battery voltage in volts, 0 if unknown
float32 voltage_filtered_v	# Battery voltage in volts, filtered, 0 if unknown
float32 current_a		# Battery current in amperes, -1 if unknown
float32 current_filtered_a	# Battery current in amperes, filtered, 0 if unknown
float32 discharged_mah		# Discharged amount in mAh, -1 if unknown
float32 remaining		# From 1 to 0, -1 if unknown
float32 scale		# Power scaling factor, >= 1, or -1 if unknown
int32 cell_count		# Number of cells
bool connected			# Whether or not a battery is connected, based on a voltage threshold
bool system_source		# Whether or not a this battery is the active power source for VDD_5V_IN
uint8 priority        	# Zerro based priority is the connection on the Power Controller V1..Vn AKA BrickN-1

#bool is_powering_off		# Power off event imminent indication, false if unknown


uint8 BATTERY_WARNING_NONE = 0       # no battery low voltage warning active
uint8 BATTERY_WARNING_LOW = 1        # warning of low voltage
uint8 BATTERY_WARNING_CRITICAL = 2   # critical voltage, return / abort immediately
uint8 BATTERY_WARNING_EMERGENCY = 3   # immediate landing required
uint8 BATTERY_WARNING_FAILED = 4   # the battery has failed completely

uint8 warning    # current battery warning
