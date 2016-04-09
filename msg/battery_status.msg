uint64 timestamp		# microseconds since system boot, needed to integrate
float32 voltage_v		# Battery voltage in volts, 0 if unknown
float32 voltage_filtered_v	# Battery voltage in volts, filtered, 0 if unknown
float32 current_a		# Battery current in amperes, -1 if unknown
float32 discharged_mah		# Discharged amount in mAh, -1 if unknown
bool is_powering_off		# Power off event imminent indication, false if unknown
