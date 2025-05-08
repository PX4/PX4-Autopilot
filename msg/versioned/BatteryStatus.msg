# Battery status

uint32 MESSAGE_VERSION = 0

uint64 timestamp # Time since system start [us]
bool connected # Whether or not a battery is connected, based on a voltage threshold
float32 voltage_v # Battery voltage [V] [@invalid 0]
float32 current_a # Battery current [A] [@invalid -1]
float32 current_average_a # Battery current average (for FW average in level flight) [A] [@invalid -1]
float32 discharged_mah # Discharged amount [mAh] [@invalid -1]
float32 remaining # Remaining capacity [@range 0,1] [@invalid -1]
float32 scale # Power scaling factor [@range 1,] [@invalid -1]
float32 time_remaining_s # Predicted time remaining until battery is empty under previous averaged load [s] [@invalid NaN]
float32 temperature # Temperature of the battery [°C] [@invalid NaN]
uint8 cell_count # Number of cells [@invalid 0]


uint8 source # Battery source [@enum SOURCE]
uint8 SOURCE_POWER_MODULE = 0 # Power module
uint8 SOURCE_EXTERNAL = 1 # External
uint8 SOURCE_ESCS = 2 # ESCs

uint8 priority # Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1
uint16 capacity # Actual capacity of the battery [mAh]
uint16 cycle_count # Number of discharge cycles the battery has experienced
uint16 average_time_to_empty # Predicted remaining battery capacity based on the average rate of discharge [minutes]
uint16 serial_number # Serial number of the battery pack
uint16 manufacture_date # Manufacture date, part of serial number of the battery pack. Formatted as: Day + Month×32 + (Year–1980)×512
uint16 state_of_health # State of health. FullChargeCapacity/DesignCapacity [%] [@range 0, 100].
uint16 max_error # Max error, expected margin of error in the state-of-charge calculation [%] [@range 1, 100]
uint8 id # ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed.
uint16 interface_error # Interface error counter

float32[14] voltage_cell_v # Battery individual cell voltages [V] [@invalid 0]
float32 max_cell_voltage_delta # Max difference between individual cell voltages

bool is_powering_off # Power off event imminent indication, false if unknown
bool is_required # Set if the battery is explicitly required before arming

uint8 warning # Current battery warning [@enum WARNING STATE]
uint8 WARNING_NONE = 0 # No battery low voltage warning active
uint8 WARNING_LOW = 1 # Low voltage warning
uint8 WARNING_CRITICAL = 2 # Critical voltage, return / abort immediately
uint8 WARNING_EMERGENCY = 3 # Immediate landing required
uint8 WARNING_FAILED = 4 # Battery has failed completely

uint8 STATE_UNHEALTHY = 6 # Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in faults field.
uint8 STATE_CHARGING = 7 # Battery is charging

uint16 faults # Smart battery supply status/fault flags (bitmask) for health indication. [@enum FAULT]
uint8 FAULT_DEEP_DISCHARGE = 0 # Battery has deep discharged
uint8 FAULT_SPIKES = 1 # Voltage spikes
uint8 FAULT_CELL_FAIL= 2 # One or more cells have failed
uint8 FAULT_OVER_CURRENT = 3 # Over-current
uint8 FAULT_OVER_TEMPERATURE = 4 # Over-temperature
uint8 FAULT_UNDER_TEMPERATURE = 5 # Under-temperature fault
uint8 FAULT_INCOMPATIBLE_VOLTAGE = 6 # Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage).
uint8 FAULT_INCOMPATIBLE_FIRMWARE = 7 # Battery firmware is not compatible with current autopilot firmware
uint8 FAULT_INCOMPATIBLE_MODEL = 8 # Battery model is not supported by the system
uint8 FAULT_HARDWARE_FAILURE = 9 # hardware problem
uint8 FAULT_FAILED_TO_ARM = 10 # Battery had a problem while arming
uint8 FAULT_COUNT = 11 # Counter - keep it as last element!

uint8 MAX_INSTANCES = 4

float32 full_charge_capacity_wh # The compensated battery capacity [Wh]
float32 remaining_capacity_wh # The compensated battery capacity remaining [Wh]
uint16 over_discharge_count # Number of battery overdischarge
float32 nominal_voltage # Nominal voltage of the battery pack [V]

float32 internal_resistance_estimate # Internal resistance per cell estimate [Ohm]
float32 ocv_estimate # Open circuit voltage estimate [V]
float32 ocv_estimate_filtered # Filtered open circuit voltage estimate [V] 
float32 volt_based_soc_estimate # Normalized volt based state of charge estimate [@range 0, 1] 
float32 voltage_prediction # Predicted voltage [V] 
float32 prediction_error # Prediction error [V] 
float32 estimation_covariance_norm # Norm of the covariance matrix
