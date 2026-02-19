---
pageClass: is-wide-page
---

# BatteryStatusV0 (UORB message)

Battery status.

Battery status information for up to 4 battery instances.
These are populated from power module and smart battery device drivers, and one battery updated from MAVLink.
Battery instance information is also logged and streamed in MAVLink telemetry.

**TOPICS:** battery_statusv0

## Fields

| 参数名                                                                                    | 类型            | Unit [Frame] | Range/Enum                                                                    | 描述                                                                                                                                                             |
| -------------------------------------------------------------------------------------- | ------------- | ---------------------------------------------------------------- | ----------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                              | `uint64`      | us                                                               |                                                                               | Time since system start                                                                                                                                        |
| connected                                                                              | `bool`        |                                                                  |                                                                               | Whether or not a battery is connected. For power modules this is based on a voltage threshold.                                 |
| voltage_v                                                         | `float32`     | V                                                                |                                                                               | Battery voltage (Invalid: 0)                                                                                                |
| current_a                                                         | `float32`     | A                                                                |                                                                               | Battery current (Invalid: -1)                                                                                               |
| current_average_a                            | `float32`     | A                                                                |                                                                               | Battery current average (for FW average in level flight) (Invalid: -1)                                   |
| discharged_mah                                                    | `float32`     | mAh                                                              |                                                                               | Discharged amount (Invalid: -1)                                                                                             |
| remaining                                                                              | `float32`     |                                                                  | [0 : 1]   | Remaining capacity (Invalid: -1)                                                                                            |
| scale                                                                                  | `float32`     |                                                                  | [1 : -]   | Scaling factor to compensate for lower actuation power caused by voltage sag (Invalid: -1)                                  |
| time_remaining_s                             | `float32`     | s                                                                |                                                                               | Predicted time remaining until battery is empty under previous averaged load (Invalid: NaN)                                 |
| temperature                                                                            | `float32`     | °C                                                               |                                                                               | Temperature of the battery (Invalid: NaN)                                                                                   |
| cell_count                                                        | `uint8`       |                                                                  |                                                                               | Number of cells (Invalid: 0)                                                                                                |
| source                                                                                 | `uint8`       |                                                                  | [SOURCE](#SOURCE)                                                             | Battery source                                                                                                                                                 |
| priority                                                                               | `uint8`       |                                                                  |                                                                               | Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1                                              |
| capacity                                                                               | `uint16`      | mAh                                                              |                                                                               | Capacity of the battery when fully charged                                                                                                                     |
| cycle_count                                                       | `uint16`      |                                                                  |                                                                               | Number of discharge cycles the battery has experienced                                                                                                         |
| average_time_to_empty   | `uint16`      | minutes                                                          |                                                                               | Predicted remaining battery capacity based on the average rate of discharge                                                                                    |
| serial_number                                                     | `uint16`      |                                                                  |                                                                               | Serial number of the battery pack                                                                                                                              |
| manufacture_date                                                  | `uint16`      |                                                                  |                                                                               | Manufacture date, part of serial number of the battery pack. Formatted as: Day + Month×32 + (Year–1980)×512 |
| state_of_health                              | `uint16`      | %                                                                | [0 : 100] | State of health. FullChargeCapacity/DesignCapacity                                                                                             |
| max_error                                                         | `uint16`      | %                                                                | [1 : 100] | Max error, expected margin of error in the state-of-charge calculation                                                                                         |
| id                                                                                     | `uint8`       |                                                                  |                                                                               | ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed                               |
| interface_error                                                   | `uint16`      |                                                                  |                                                                               | Interface error counter                                                                                                                                        |
| voltage_cell_v                               | `float32[14]` | V                                                                |                                                                               | Battery individual cell voltages (Invalid: 0)                                                                               |
| max_cell_voltage_delta  | `float32`     |                                                                  |                                                                               | Max difference between individual cell voltages                                                                                                                |
| is_powering_off                              | `bool`        |                                                                  |                                                                               | Power off event imminent indication, false if unknown                                                                                                          |
| is_required                                                       | `bool`        |                                                                  |                                                                               | Set if the battery is explicitly required before arming                                                                                                        |
| warning                                                                                | `uint8`       |                                                                  | [WARNING](#WARNING)[STATE](#STATE)                                            | Current battery warning                                                                                                                                        |
| faults                                                                                 | `uint16`      |                                                                  | [FAULT](#FAULT)                                                               | Smart battery supply status/fault flags (bitmask) for health indication                                                                     |
| full_charge_capacity_wh | `float32`     | Wh                                                               |                                                                               | Compensated battery capacity                                                                                                                                   |
| remaining_capacity_wh                        | `float32`     | Wh                                                               |                                                                               | Compensated battery capacity remaining                                                                                                                         |
| over_discharge_count                         | `uint16`      |                                                                  |                                                                               | Number of battery overdischarge                                                                                                                                |
| nominal_voltage                                                   | `float32`     | V                                                                |                                                                               | Nominal voltage of the battery pack                                                                                                                            |
| internal_resistance_estimate                 | `float32`     | Ohm                                                              |                                                                               | Internal resistance per cell estimate                                                                                                                          |
| ocv_estimate                                                      | `float32`     | V                                                                |                                                                               | Open circuit voltage estimate                                                                                                                                  |
| ocv_estimate_filtered                        | `float32`     | V                                                                |                                                                               | Filtered open circuit voltage estimate                                                                                                                         |
| volt_based_soc_estimate | `float32`     |                                                                  | [0 : 1]   | Normalized volt based state of charge estimate                                                                                                                 |
| voltage_prediction                                                | `float32`     | V                                                                |                                                                               | Predicted voltage                                                                                                                                              |
| prediction_error                                                  | `float32`     | V                                                                |                                                                               | Prediction error                                                                                                                                               |
| estimation_covariance_norm                   | `float32`     |                                                                  |                                                                               | Norm of the covariance matrix                                                                                                                                  |

## Enums

### SOURCE {#SOURCE}

| 参数名                                                                                               | 类型      | 值 | 描述                                                                |
| ------------------------------------------------------------------------------------------------- | ------- | - | ----------------------------------------------------------------- |
| <a href="#SOURCE_POWER_MODULE"></a> SOURCE_POWER_MODULE | `uint8` | 0 | Power module (analog ADC or I2C power monitor) |
| <a href="#SOURCE_EXTERNAL"></a> SOURCE_EXTERNAL                              | `uint8` | 1 | External (MAVLink, CAN, or external driver)    |
| <a href="#SOURCE_ESCS"></a> SOURCE_ESCS                                      | `uint8` | 2 | ESCs (via ESC telemetry)                       |

### WARNING {#WARNING}

| 参数名                                                                      | 类型      | 值 | 描述                                           |
| ------------------------------------------------------------------------ | ------- | - | -------------------------------------------- |
| <a href="#WARNING_NONE"></a> WARNING_NONE           | `uint8` | 0 | No battery low voltage warning active        |
| <a href="#WARNING_LOW"></a> WARNING_LOW             | `uint8` | 1 | Low voltage warning                          |
| <a href="#WARNING_CRITICAL"></a> WARNING_CRITICAL   | `uint8` | 2 | Critical voltage, return / abort immediately |
| <a href="#WARNING_EMERGENCY"></a> WARNING_EMERGENCY | `uint8` | 3 | Immediate landing required                   |
| <a href="#WARNING_FAILED"></a> WARNING_FAILED       | `uint8` | 4 | Battery has failed completely                |

### STATE {#STATE}

| 参数名                                                                  | 类型      | 值 | 描述                                                                                                                                                                                   |
| -------------------------------------------------------------------- | ------- | - | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a href="#STATE_UNHEALTHY"></a> STATE_UNHEALTHY | `uint8` | 6 | Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in faults field |
| <a href="#STATE_CHARGING"></a> STATE_CHARGING   | `uint8` | 7 | Battery is charging                                                                                                                                                                  |

### FAULT {#FAULT}

| 参数名                                                                                                                    | 类型      | 值  | 描述                                                                                                                                |
| ---------------------------------------------------------------------------------------------------------------------- | ------- | -- | --------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#FAULT_DEEP_DISCHARGE"></a> FAULT_DEEP_DISCHARGE                    | `uint8` | 0  | Battery has deep discharged                                                                                                       |
| <a href="#FAULT_SPIKES"></a> FAULT_SPIKES                                                         | `uint8` | 1  | Voltage spikes                                                                                                                    |
| <a href="#FAULT_CELL_FAIL"></a> FAULT_CELL_FAIL                              | `uint8` | 2  | One or more cells have failed                                                                                                     |
| <a href="#FAULT_OVER_CURRENT"></a> FAULT_OVER_CURRENT                        | `uint8` | 3  | Over-current                                                                                                                      |
| <a href="#FAULT_OVER_TEMPERATURE"></a> FAULT_OVER_TEMPERATURE                | `uint8` | 4  | Over-temperature                                                                                                                  |
| <a href="#FAULT_UNDER_TEMPERATURE"></a> FAULT_UNDER_TEMPERATURE              | `uint8` | 5  | Under-temperature fault                                                                                                           |
| <a href="#FAULT_INCOMPATIBLE_VOLTAGE"></a> FAULT_INCOMPATIBLE_VOLTAGE        | `uint8` | 6  | Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage) |
| <a href="#FAULT_INCOMPATIBLE_FIRMWARE"></a> FAULT_INCOMPATIBLE_FIRMWARE      | `uint8` | 7  | Battery firmware is not compatible with current autopilot firmware                                                                |
| <a href="#FAULT_INCOMPATIBLE_MODEL"></a> FAULT_INCOMPATIBLE_MODEL            | `uint8` | 8  | Battery model is not supported by the system                                                                                      |
| <a href="#FAULT_HARDWARE_FAILURE"></a> FAULT_HARDWARE_FAILURE                | `uint8` | 9  | Hardware problem                                                                                                                  |
| <a href="#FAULT_FAILED_TO_ARM"></a> FAULT_FAILED_TO_ARM | `uint8` | 10 | Battery had a problem while arming                                                                                                |
| <a href="#FAULT_COUNT"></a> FAULT_COUNT                                                           | `uint8` | 11 | Counter. Keep this as last element                                                                                |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述 |
| -------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |    |
| <a href="#MAX_INSTANCES"></a> MAX_INSTANCES     | `uint8`  | 4 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/BatteryStatusV0.msg)

:::details
Click here to see original file

```c
# Battery status
#
# Battery status information for up to 4 battery instances.
# These are populated from power module and smart battery device drivers, and one battery updated from MAVLink.
# Battery instance information is also logged and streamed in MAVLink telemetry.

uint32 MESSAGE_VERSION = 0
uint8 MAX_INSTANCES = 4

uint64 timestamp # [us] Time since system start
bool connected # Whether or not a battery is connected. For power modules this is based on a voltage threshold.
float32 voltage_v # [V] [@invalid 0] Battery voltage
float32 current_a # [A] [@invalid -1] Battery current
float32 current_average_a # [A] [@invalid -1] Battery current average (for FW average in level flight)
float32 discharged_mah # [mAh] [@invalid -1] Discharged amount
float32 remaining # [@range 0,1] [@invalid -1] Remaining capacity
float32 scale # [@range 1,] [@invalid -1] Scaling factor to compensate for lower actuation power caused by voltage sag
float32 time_remaining_s # [s] [@invalid NaN] Predicted time remaining until battery is empty under previous averaged load
float32 temperature # [°C] [@invalid NaN] Temperature of the battery
uint8 cell_count # [@invalid 0] Number of cells


uint8 source # [@enum SOURCE] Battery source
uint8 SOURCE_POWER_MODULE = 0 # Power module (analog ADC or I2C power monitor)
uint8 SOURCE_EXTERNAL = 1 # External (MAVLink, CAN, or external driver)
uint8 SOURCE_ESCS = 2 # ESCs (via ESC telemetry)

uint8 priority # Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1
uint16 capacity # [mAh] Capacity of the battery when fully charged
uint16 cycle_count # Number of discharge cycles the battery has experienced
uint16 average_time_to_empty # [minutes] Predicted remaining battery capacity based on the average rate of discharge
uint16 serial_number # Serial number of the battery pack
uint16 manufacture_date # Manufacture date, part of serial number of the battery pack. Formatted as: Day + Month×32 + (Year–1980)×512
uint16 state_of_health # [%] [@range 0, 100] State of health. FullChargeCapacity/DesignCapacity
uint16 max_error # [%] [@range 1, 100] Max error, expected margin of error in the state-of-charge calculation
uint8 id # ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed
uint16 interface_error # Interface error counter

float32[14] voltage_cell_v # [V] [@invalid 0] Battery individual cell voltages
float32 max_cell_voltage_delta # Max difference between individual cell voltages

bool is_powering_off # Power off event imminent indication, false if unknown
bool is_required # Set if the battery is explicitly required before arming

uint8 warning # [@enum WARNING STATE] Current battery warning
uint8 WARNING_NONE = 0 # No battery low voltage warning active
uint8 WARNING_LOW = 1 # Low voltage warning
uint8 WARNING_CRITICAL = 2 # Critical voltage, return / abort immediately
uint8 WARNING_EMERGENCY = 3 # Immediate landing required
uint8 WARNING_FAILED = 4 # Battery has failed completely
uint8 STATE_UNHEALTHY = 6 # Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in faults field
uint8 STATE_CHARGING = 7 # Battery is charging

uint16 faults # [@enum FAULT] Smart battery supply status/fault flags (bitmask) for health indication
uint8 FAULT_DEEP_DISCHARGE = 0 # Battery has deep discharged
uint8 FAULT_SPIKES = 1 # Voltage spikes
uint8 FAULT_CELL_FAIL= 2 # One or more cells have failed
uint8 FAULT_OVER_CURRENT = 3 # Over-current
uint8 FAULT_OVER_TEMPERATURE = 4 # Over-temperature
uint8 FAULT_UNDER_TEMPERATURE = 5 # Under-temperature fault
uint8 FAULT_INCOMPATIBLE_VOLTAGE = 6 # Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage)
uint8 FAULT_INCOMPATIBLE_FIRMWARE = 7 # Battery firmware is not compatible with current autopilot firmware
uint8 FAULT_INCOMPATIBLE_MODEL = 8 # Battery model is not supported by the system
uint8 FAULT_HARDWARE_FAILURE = 9 # Hardware problem
uint8 FAULT_FAILED_TO_ARM = 10 # Battery had a problem while arming
uint8 FAULT_COUNT = 11 # Counter. Keep this as last element

float32 full_charge_capacity_wh # [Wh] Compensated battery capacity
float32 remaining_capacity_wh # [Wh] Compensated battery capacity remaining
uint16 over_discharge_count # Number of battery overdischarge
float32 nominal_voltage # [V] Nominal voltage of the battery pack

float32 internal_resistance_estimate # [Ohm] Internal resistance per cell estimate
float32 ocv_estimate # [V] Open circuit voltage estimate
float32 ocv_estimate_filtered # [V] Filtered open circuit voltage estimate
float32 volt_based_soc_estimate # [@range 0, 1] Normalized volt based state of charge estimate
float32 voltage_prediction # [V] Predicted voltage
float32 prediction_error # [V] Prediction error
float32 estimation_covariance_norm # Norm of the covariance matrix
```

:::
