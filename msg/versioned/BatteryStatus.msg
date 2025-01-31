uint32 MESSAGE_VERSION = 0

uint64 timestamp				# Time since system start (microseconds)
uint8 id					# ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed

bool connected					# Whether or not a battery is connected, based on a voltage threshold
float32 voltage_v				# Battery voltage in volts, 0 if unknown
float32 current_a				# Battery current in amperes, -1 if unknown
float32 current_average_a			# Battery current average in amperes (for FW average in level flight), -1 if unknown
float32 discharged_mah				# Discharged amount in mAh, -1 if unknown
float32 remaining				# From 1 to 0, -1 if unknown
float32 scale					# Power scaling factor, >= 1, or -1 if unknown
float32 time_remaining_s			# Predicted time in seconds remaining until battery is empty under previous averaged load, NAN if unknown
float32 temperature				# Temperature of the battery in degrees Celcius, NaN if unknown

uint8 priority					# Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1

uint16 interface_error				# Interface error counter

float32[14] voltage_cell_v			# Battery individual cell voltages, 0 if unknown
float32 max_cell_voltage_delta			# Max difference between individual cell voltages

bool is_powering_off				# Power off event imminent indication, false if unknown
bool is_required				# Set if the battery is explicitly required before arming

uint8 MAX_INSTANCES = 3

uint8 warning					# Current battery warning
uint8 WARNING_NONE = 0				# No battery low voltage warning active
uint8 BATTERY_WARNING_LOW = 1			# Warning of low voltage
uint8 BATTERY_WARNING_CRITICAL = 2		# Critical voltage, return / abort immediately
uint8 BATTERY_WARNING_EMERGENCY = 3		# Immediate landing required
uint8 BATTERY_WARNING_FAILED = 4		# The battery has failed completely
uint8 BATTERY_STATE_UNHEALTHY = 6 		# Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in faults field
uint8 BATTERY_STATE_CHARGING = 7 		# Battery is charging


uint16 faults					# Smart battery supply status/fault flags (bitmask) for health indication
uint8 BATTERY_FAULT_DEEP_DISCHARGE = 0 		# Battery has deep discharged
uint8 BATTERY_FAULT_SPIKES = 1 			# Voltage spikes
uint8 BATTERY_FAULT_CELL_FAIL= 2 		# One or more cells have failed
uint8 BATTERY_FAULT_OVER_CURRENT = 3 		# Over-current
uint8 BATTERY_FAULT_OVER_TEMPERATURE = 4 	# Over-temperature
uint8 BATTERY_FAULT_UNDER_TEMPERATURE = 5 	# Under-temperature fault
uint8 BATTERY_FAULT_INCOMPATIBLE_VOLTAGE = 6 	# Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage)
uint8 BATTERY_FAULT_INCOMPATIBLE_FIRMWARE = 7 	# Battery firmware is not compatible with current autopilot firmware
uint8 BATTERY_FAULT_INCOMPATIBLE_MODEL = 8 	# Battery model is not supported by the system
uint8 BATTERY_FAULT_HARDWARE_FAILURE = 9 	# hardware problem
uint8 BATTERY_FAULT_FAILED_TO_ARM = 10 		# Battery had a problem while arming
uint8 BATTERY_FAULT_COUNT = 11 			# Counter - keep it as last element!


float32 ocv_estimate                            # [V] Open circuit voltage estimate
float32 ocv_estimate_filtered			# [V] Filtered open circuit voltage estimate
float32 volt_based_soc_estimate			# [0, 1] Normalized Volt-based state of charge estimate
float32 voltage_prediction			# [V] Predicted voltage
float32 prediction_error                        # [V] Prediction error
float32 estimation_covariance_norm		# Norm of the covariance matrix



# to be removed
uint8 cell_count				# Number of cells, 0 if unknown

uint8 source					# Battery source
uint8 SOURCE_POWER_MODULE = 0			# Source is a power module
uint8 SOURCE_EXTERNAL = 1			# Source is external
uint8 SOURCE_ESCS = 2				# Source is an ESC

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
