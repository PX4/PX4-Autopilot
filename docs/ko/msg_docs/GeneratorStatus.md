---
pageClass: is-wide-page
---

# GeneratorStatus (UORB message)

**TOPICS:** generator_status

## Fields

| 명칭                                                               | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                                                                                               |
| ---------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| timestamp                                                        | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                        |
| status                                                           | `uint64`  |                                                                  |            | Status flags                                                                                                                                                                                                                                     |
| battery_current                             | `float32` | A                                                                |            | Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.                                                                         |
| load_current                                | `float32` | A                                                                |            | Current going to the UAV. If battery current not available this is the DC current from the generator. Positive for out. Negative for in. NaN: field not provided |
| power_generated                             | `float32` | W                                                                |            | The power being generated. NaN: field not provided                                                                                                                                                               |
| bus_voltage                                 | `float32` | V                                                                |            | Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and at a different voltage to main bus.                                                                                       |
| bat_current_setpoint   | `float32` | A                                                                |            | The target battery current. Positive for out. Negative for in. NaN: field not provided                                                                                           |
| runtime                                                          | `uint32`  | s                                                                |            | Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.                                                                                       |
| time_until_maintenance | `int32`   | s                                                                |            | Seconds until this generator requires maintenance. A negative value indicates maintenance is past-due. INT32_MAX: field not provided.                       |
| generator_speed                             | `uint16`  | rpm                                                              |            | Speed of electrical generator or alternator. UINT16_MAX: field not provided.                                                                                                |
| rectifier_temperature                       | `int16`   | degC                                                             |            | The temperature of the rectifier or power converter. INT16_MAX: field not provided.                                                                                         |
| generator_temperature                       | `int16`   | degC                                                             |            | The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.                                                                        |

## Constants

| 명칭                                                                                                                                                                                                                 | 형식       | Value   | 설명                                                                                                                                                                                                                                     |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#STATUS_FLAG_OFF"></a> STATUS_FLAG_OFF                                                                                                                          | `uint64` | 1       | Generator is off.                                                                                                                                                                                                      |
| <a href="#STATUS_FLAG_READY"></a> STATUS_FLAG_READY                                                                                                                      | `uint64` | 2       | Generator is ready to start generating power.                                                                                                                                                                          |
| <a href="#STATUS_FLAG_GENERATING"></a> STATUS_FLAG_GENERATING                                                                                                            | `uint64` | 4       | Generator is generating power.                                                                                                                                                                                         |
| <a href="#STATUS_FLAG_CHARGING"></a> STATUS_FLAG_CHARGING                                                                                                                | `uint64` | 8       | Generator is charging the batteries (generating enough power to charge and provide the load).                                                                                                       |
| <a href="#STATUS_FLAG_REDUCED_POWER"></a> STATUS_FLAG_REDUCED_POWER                                                                                 | `uint64` | 16      | Generator is operating at a reduced maximum power.                                                                                                                                                                     |
| <a href="#STATUS_FLAG_MAXPOWER"></a> STATUS_FLAG_MAXPOWER                                                                                                                | `uint64` | 32      | Generator is providing the maximum output.                                                                                                                                                                             |
| <a href="#STATUS_FLAG_OVERTEMP_WARNING"></a> STATUS_FLAG_OVERTEMP_WARNING                                                                           | `uint64` | 64      | Generator is near the maximum operating temperature, cooling is insufficient.                                                                                                                                          |
| <a href="#STATUS_FLAG_OVERTEMP_FAULT"></a> STATUS_FLAG_OVERTEMP_FAULT                                                                               | `uint64` | 128     | Generator hit the maximum operating temperature and shutdown.                                                                                                                                                          |
| <a href="#STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING"></a> STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING                              | `uint64` | 256     | Power electronics are near the maximum operating temperature, cooling is insufficient.                                                                                                                                 |
| <a href="#STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT"></a> STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT                                  | `uint64` | 512     | Power electronics hit the maximum operating temperature and shutdown.                                                                                                                                                  |
| <a href="#STATUS_FLAG_ELECTRONICS_FAULT"></a> STATUS_FLAG_ELECTRONICS_FAULT                                                                         | `uint64` | 1024    | Power electronics experienced a fault and shutdown.                                                                                                                                                                    |
| <a href="#STATUS_FLAG_POWERSOURCE_FAULT"></a> STATUS_FLAG_POWERSOURCE_FAULT                                                                         | `uint64` | 2048    | The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening. |
| <a href="#STATUS_FLAG_COMMUNICATION_WARNING"></a> STATUS_FLAG_COMMUNICATION_WARNING                                                                 | `uint64` | 4096    | Generator controller having communication problems.                                                                                                                                                                    |
| <a href="#STATUS_FLAG_COOLING_WARNING"></a> STATUS_FLAG_COOLING_WARNING                                                                             | `uint64` | 8192    | Power electronic or generator cooling system error.                                                                                                                                                                    |
| <a href="#STATUS_FLAG_POWER_RAIL_FAULT"></a> STATUS_FLAG_POWER_RAIL_FAULT                                                      | `uint64` | 16384   | Generator controller power rail experienced a fault.                                                                                                                                                                   |
| <a href="#STATUS_FLAG_OVERCURRENT_FAULT"></a> STATUS_FLAG_OVERCURRENT_FAULT                                                                         | `uint64` | 32768   | Generator controller exceeded the overcurrent threshold and shutdown to prevent damage.                                                                                                                                |
| <a href="#STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT"></a> STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT | `uint64` | 65536   | Generator controller detected a high current going into the batteries and shutdown to prevent battery damage.                                                                                                          |
| <a href="#STATUS_FLAG_OVERVOLTAGE_FAULT"></a> STATUS_FLAG_OVERVOLTAGE_FAULT                                                                         | `uint64` | 131072  | Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage rating.                                                                                                      |
| <a href="#STATUS_FLAG_BATTERY_UNDERVOLT_FAULT"></a> STATUS_FLAG_BATTERY_UNDERVOLT_FAULT                                        | `uint64` | 262144  | Batteries are under voltage (generator will not start).                                                                                                                                             |
| <a href="#STATUS_FLAG_START_INHIBITED"></a> STATUS_FLAG_START_INHIBITED                                                                             | `uint64` | 524288  | Generator start is inhibited by e.g. a safety switch.                                                                                                                                  |
| <a href="#STATUS_FLAG_MAINTENANCE_REQUIRED"></a> STATUS_FLAG_MAINTENANCE_REQUIRED                                                                   | `uint64` | 1048576 | Generator requires maintenance.                                                                                                                                                                                        |
| <a href="#STATUS_FLAG_WARMING_UP"></a> STATUS_FLAG_WARMING_UP                                                                                       | `uint64` | 2097152 | Generator is not ready to generate yet.                                                                                                                                                                                |
| <a href="#STATUS_FLAG_IDLE"></a> STATUS_FLAG_IDLE                                                                                                                        | `uint64` | 4194304 | Generator is idle.                                                                                                                                                                                                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GeneratorStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp			# time since system start (microseconds)


uint64 STATUS_FLAG_OFF                              = 1       # Generator is off.
uint64 STATUS_FLAG_READY                            = 2       # Generator is ready to start generating power.
uint64 STATUS_FLAG_GENERATING                       = 4       # Generator is generating power.
uint64 STATUS_FLAG_CHARGING                         = 8       # Generator is charging the batteries (generating enough power to charge and provide the load).
uint64 STATUS_FLAG_REDUCED_POWER                    = 16      # Generator is operating at a reduced maximum power.
uint64 STATUS_FLAG_MAXPOWER                         = 32      # Generator is providing the maximum output.
uint64 STATUS_FLAG_OVERTEMP_WARNING                 = 64      # Generator is near the maximum operating temperature, cooling is insufficient.
uint64 STATUS_FLAG_OVERTEMP_FAULT                   = 128     # Generator hit the maximum operating temperature and shutdown.
uint64 STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING     = 256     # Power electronics are near the maximum operating temperature, cooling is insufficient.
uint64 STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT       = 512     # Power electronics hit the maximum operating temperature and shutdown.
uint64 STATUS_FLAG_ELECTRONICS_FAULT                = 1024    # Power electronics experienced a fault and shutdown.
uint64 STATUS_FLAG_POWERSOURCE_FAULT                = 2048    # The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening.
uint64 STATUS_FLAG_COMMUNICATION_WARNING            = 4096    # Generator controller having communication problems.
uint64 STATUS_FLAG_COOLING_WARNING                  = 8192    # Power electronic or generator cooling system error.
uint64 STATUS_FLAG_POWER_RAIL_FAULT                 = 16384   # Generator controller power rail experienced a fault.
uint64 STATUS_FLAG_OVERCURRENT_FAULT                = 32768   # Generator controller exceeded the overcurrent threshold and shutdown to prevent damage.
uint64 STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT = 65536   # Generator controller detected a high current going into the batteries and shutdown to prevent battery damage. |
uint64 STATUS_FLAG_OVERVOLTAGE_FAULT                = 131072  # Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage rating.
uint64 STATUS_FLAG_BATTERY_UNDERVOLT_FAULT          = 262144  # Batteries are under voltage (generator will not start).
uint64 STATUS_FLAG_START_INHIBITED                  = 524288  # Generator start is inhibited by e.g. a safety switch.
uint64 STATUS_FLAG_MAINTENANCE_REQUIRED             = 1048576 # Generator requires maintenance.
uint64 STATUS_FLAG_WARMING_UP                       = 2097152 # Generator is not ready to generate yet.
uint64 STATUS_FLAG_IDLE                             = 4194304 # Generator is idle.

uint64 status                      # Status flags


float32 battery_current            # [A] Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.
float32 load_current               # [A] Current going to the UAV. If battery current not available this is the DC current from the generator. Positive for out. Negative for in. NaN: field not provided
float32 power_generated            # [W] The power being generated. NaN: field not provided
float32 bus_voltage                # [V] Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and at a different voltage to main bus.
float32 bat_current_setpoint       # [A] The target battery current. Positive for out. Negative for in. NaN: field not provided

uint32 runtime                     # [s] Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.

int32 time_until_maintenance       # [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past-due. INT32_MAX: field not provided.

uint16 generator_speed             # [rpm] Speed of electrical generator or alternator. UINT16_MAX: field not provided.

int16 rectifier_temperature        # [degC] The temperature of the rectifier or power converter. INT16_MAX: field not provided.
int16 generator_temperature        # [degC] The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.
```

:::
