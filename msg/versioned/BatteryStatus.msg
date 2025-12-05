# Battery status
#
# Battery status information for up to 3 battery instances.
# These are populated from power module and smart battery device drivers, and one battery updated from MAVLink.
# Battery instance information is also logged and streamed in MAVLink telemetry.

uint32 MESSAGE_VERSION = 1
uint8 MAX_INSTANCES = 3

uint64 timestamp  # [us] Time since system start

bool connected             # Whether or not a battery is connected. For power modules this is based on a voltage threshold.
float32 voltage_v          # [V] [@invalid 0] Battery voltage
float32 current_a          # [A] [@invalid -1] Battery current
float32 current_average_a  # [A] [@invalid -1] Battery current average (for FW average in level flight)
float32 discharged_mah     # [mAh] [@invalid -1] Discharged amount
float32 remaining          # [@range 0,1] [@invalid -1] Remaining capacity
float32 scale              # [-] [@range 1,] [@invalid -1] Scaling factor to compensate for lower actuation power caused by voltage sag
float32 time_remaining_s   # [s] [@invalid NaN] Predicted time remaining until battery is empty under previous averaged load
float32 temperature        # [°C] [@invalid NaN] Temperature of the battery
uint8 cell_count           # [-] [@invalid 0] Number of cells


uint8 source                   # [@enum SOURCE] Battery source
uint8 SOURCE_POWER_MODULE = 0  # Power module
uint8 SOURCE_EXTERNAL = 1      # External
uint8 SOURCE_ESCS = 2          # ESCs

uint8 priority                # [-] Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1
uint16 capacity               # [mAh] Capacity of the battery when fully charged
uint16 cycle_count            # [-] Number of discharge cycles the battery has experienced
uint16 average_time_to_empty  # [minutes] Predicted remaining battery capacity based on the average rate of discharge
uint16 manufacture_date       # [-] Manufacture date, part of serial number of the battery pack. Formatted as: Day + Month×32 + (Year–1980)×512
uint16 state_of_health        # [%] [@range 0, 100] State of health. FullChargeCapacity/DesignCapacity
uint16 max_error              # [%] [@range 1, 100] Max error, expected margin of error in the state-of-charge calculation
uint8 id                      # [-] ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed
uint16 interface_error        # [-] Interface error counter

float32[14] voltage_cell_v      # [V] [@invalid 0] Battery individual cell voltages
float32 max_cell_voltage_delta  # [V] Max difference between individual cell voltages

bool is_powering_off  # Power off event imminent indication, false if unknown
bool is_required      # Set if the battery is explicitly required before arming

uint8 warning                # [@enum WARNING STATE] Current battery warning
uint8 WARNING_NONE = 0       # No battery low voltage warning active
uint8 WARNING_LOW = 1        # Low voltage warning
uint8 WARNING_CRITICAL = 2   # Critical voltage, return / abort immediately
uint8 WARNING_EMERGENCY = 3  # Immediate landing required
uint8 WARNING_FAILED = 4     # Battery has failed completely
uint8 STATE_UNHEALTHY = 6    # Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in faults field
uint8 STATE_CHARGING = 7     # Battery is charging

uint16 faults                          # [@enum FAULT] Smart battery supply status/fault flags (bitmask) for health indication
uint8 FAULT_DEEP_DISCHARGE = 0         # Battery has deep discharged
uint8 FAULT_SPIKES = 1                 # Voltage spikes
uint8 FAULT_CELL_FAIL= 2               # One or more cells have failed
uint8 FAULT_OVER_CURRENT = 3           # Over-current
uint8 FAULT_OVER_TEMPERATURE = 4       # Over-temperature
uint8 FAULT_UNDER_TEMPERATURE = 5      # Under-temperature fault
uint8 FAULT_INCOMPATIBLE_VOLTAGE = 6   # Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage)
uint8 FAULT_INCOMPATIBLE_FIRMWARE = 7  # Battery firmware is not compatible with current autopilot firmware
uint8 FAULT_INCOMPATIBLE_MODEL = 8     # Battery model is not supported by the system
uint8 FAULT_HARDWARE_FAILURE = 9       # Hardware problem
uint8 FAULT_FAILED_TO_ARM = 10         # Battery had a problem while arming
uint8 FAULT_COUNT = 11                 # Counter. Keep this as last element

float32 full_charge_capacity_wh  # [Wh] Compensated battery capacity
float32 remaining_capacity_wh    # [Wh] Compensated battery capacity remaining
uint16 over_discharge_count      # [-] Number of battery overdischarge
float32 nominal_voltage          # [V] Nominal voltage of the battery pack

float32 internal_resistance_estimate  # [Ohm] Internal resistance per cell estimate
float32 ocv_estimate                  # [V] Open circuit voltage estimate
float32 ocv_estimate_filtered         # [V] Filtered open circuit voltage estimate
float32 volt_based_soc_estimate       # [-] [@range 0, 1] Normalized volt based state of charge estimate
float32 voltage_prediction            # [V] Predicted voltage
float32 prediction_error              # [V] Prediction error
float32 estimation_covariance_norm    # [-] Norm of the covariance matrix
