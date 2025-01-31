uint64 timestamp				# Time since system start (microseconds)
uint8 id					# ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed
uint8 cell_count				# Number of cells, 0 if unknown

uint8 source
uint8 POWER_MODULE = 0				# Source is a power module
uint8 EXTERNAL = 1				# Source is external
uint8 ESCS = 2					# Source is an ESC

uint16 capacity					# Actual capacity of the battery
uint16 cycle_count				# Number of discharge cycles the battery has experienced
uint16 average_time_to_empty			# Predicted remaining battery capacity based on the average rate of discharge (min)
char[32] serial_number				# Serial number of the battery pack
uint16 manufacture_date				# Manufacture date, part of serial number of the battery pack. Formatted as: Day + Month×32 + (Year–1980)×512
uint16 state_of_health				# State of health. FullChargeCapacity/DesignCapacity, 0-100%
uint16 max_error				# Max error, expected margin of error in % in the state-of-charge calculation with a range of 1 to 100%

float32 design_capacity_wh			# The original battery capacity
float32 full_charge_capacity_wh     		# The compensated battery capacity
float32 remaining_capacity_wh       		# The compensated battery capacity remaining
uint16 over_discharge_count         		# Number of battery overdischarge
float32 nominal_voltage             		# Nominal voltage of the battery pack

float32 internal_resistance_estimate            # Internal resistance per cell estimate (Ohm)
