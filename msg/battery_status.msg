uint64 timestamp		# microseconds since system boot, needed to integrate
float32 voltage_v		# Battery voltage in volts, 0 if unknown
float32 voltage_filtered_v	# Battery voltage in volts, filtered, 0 if unknown
float32 current_a		# Battery current in amperes, -1 if unknown
float32 discharged_mah		# Discharged amount in mAh, -1 if unknown
float32 remaining		# From 1 to 0, -1 if unknown
int32 cell_count		# Number of cells
#bool is_powering_off		# Power off event imminent indication, false if unknown


uint8 BATTERY_WARNING_NONE = 0       # no battery low voltage warning active
uint8 BATTERY_WARNING_LOW = 1        # warning of low voltage
uint8 BATTERY_WARNING_CRITICAL = 2   # alerting of critical voltage

uint8 warning    # current battery warning
