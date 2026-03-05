---
pageClass: is-wide-page
---

# InternalCombustionEngineStatus (UORB message)

**TOPICS:** internal_combustionengine_status

## Fields

| 参数名                                                                                                                    | 类型        | Unit [Frame] | Range/Enum | 描述                                                                                          |
| ---------------------------------------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------- |
| timestamp                                                                                                              | `uint64`  |                                                                  |            | time since system start (microseconds)                                   |
| state                                                                                                                  | `uint8`   |                                                                  |            |                                                                                             |
| flags                                                                                                                  | `uint32`  |                                                                  |            |                                                                                             |
| engine_load_percent                                                          | `uint8`   |                                                                  |            | Engine load estimate, percent, [0, 127] |
| engine_speed_rpm                                                             | `uint32`  |                                                                  |            | Engine speed, revolutions per minute                                                        |
| spark_dwell_time_ms                                     | `float32` |                                                                  |            | Spark dwell time, millisecond                                                               |
| atmospheric_pressure_kpa                                                     | `float32` |                                                                  |            | Atmospheric (barometric) pressure, kilopascal                            |
| intake_manifold_pressure_kpa                            | `float32` |                                                                  |            | Engine intake manifold pressure, kilopascal                                                 |
| intake_manifold_temperature                                                  | `float32` |                                                                  |            | Engine intake manifold temperature, kelvin                                                  |
| coolant_temperature                                                                               | `float32` |                                                                  |            | Engine coolant temperature, kelvin                                                          |
| oil_pressure                                                                                      | `float32` |                                                                  |            | Oil pressure, kilopascal                                                                    |
| oil_temperature                                                                                   | `float32` |                                                                  |            | Oil temperature, kelvin                                                                     |
| fuel_pressure                                                                                     | `float32` |                                                                  |            | Fuel pressure, kilopascal                                                                   |
| fuel_consumption_rate_cm3pm                             | `float32` |                                                                  |            | Instant fuel consumption estimate, (centimeter^3)/minute                 |
| estimated_consumed_fuel_volume_cm3 | `float32` |                                                                  |            | Estimate of the consumed fuel since the start of the engine, centimeter^3                   |
| throttle_position_percent                                                    | `uint8`   |                                                                  |            | Throttle position, percent                                                                  |
| ecu_index                                                                                         | `uint8`   |                                                                  |            | The index of the publishing ECU                                                             |
| spark_plug_usage                                                             | `uint8`   |                                                                  |            | Spark plug activity report.                                                 |
| ignition_timing_deg                                                          | `float32` |                                                                  |            | Cylinder ignition timing, angular degrees of the crankshaft                                 |
| injection_time_ms                                                            | `float32` |                                                                  |            | Fuel injection time, millisecond                                                            |
| cylinder_head_temperature                                                    | `float32` |                                                                  |            | Cylinder head temperature (CHT), kelvin                                  |
| exhaust_gas_temperature                                                      | `float32` |                                                                  |            | Exhaust gas temperature (EGT), kelvin                                    |
| lambda_coefficient                                                                                | `float32` |                                                                  |            | Estimated lambda coefficient, dimensionless ratio                                           |

## Constants

| 参数名                                                                                                                                                                               | 类型       | 值      | 描述                                                                                     |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | ------ | -------------------------------------------------------------------------------------- |
| <a href="#STATE_STOPPED"></a> STATE_STOPPED                                                                                                                  | `uint8`  | 0      | The engine is not running. This is the default state.  |
| <a href="#STATE_STARTING"></a> STATE_STARTING                                                                                                                | `uint8`  | 1      | The engine is starting. This is a transient state.     |
| <a href="#STATE_RUNNING"></a> STATE_RUNNING                                                                                                                  | `uint8`  | 2      | The engine is running normally.                                        |
| <a href="#STATE_FAULT"></a> STATE_FAULT                                                                                                                      | `uint8`  | 3      | The engine can no longer function.                                     |
| <a href="#FLAG_GENERAL_ERROR"></a> FLAG_GENERAL_ERROR                                                                                   | `uint32` | 1      | General error.                                                         |
| <a href="#FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED"></a> FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED | `uint32` | 2      | Error of the crankshaft sensor. This flag is optional. |
| <a href="#FLAG_CRANKSHAFT_SENSOR_ERROR"></a> FLAG_CRANKSHAFT_SENSOR_ERROR                                          | `uint32` | 4      |                                                                                        |
| <a href="#FLAG_TEMPERATURE_SUPPORTED"></a> FLAG_TEMPERATURE_SUPPORTED                                                                   | `uint32` | 8      | Temperature levels. These flags are optional                           |
| <a href="#FLAG_TEMPERATURE_BELOW_NOMINAL"></a> FLAG_TEMPERATURE_BELOW_NOMINAL                                      | `uint32` | 16     | Under-temperature warning                                                              |
| <a href="#FLAG_TEMPERATURE_ABOVE_NOMINAL"></a> FLAG_TEMPERATURE_ABOVE_NOMINAL                                      | `uint32` | 32     | Over-temperature warning                                                               |
| <a href="#FLAG_TEMPERATURE_OVERHEATING"></a> FLAG_TEMPERATURE_OVERHEATING                                                               | `uint32` | 64     | Critical overheating                                                                   |
| <a href="#FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL"></a> FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL         | `uint32` | 128    | Exhaust gas over-temperature warning                                                   |
| <a href="#FLAG_FUEL_PRESSURE_SUPPORTED"></a> FLAG_FUEL_PRESSURE_SUPPORTED                                          | `uint32` | 256    | Fuel pressure. These flags are optional                                |
| <a href="#FLAG_FUEL_PRESSURE_BELOW_NOMINAL"></a> FLAG_FUEL_PRESSURE_BELOW_NOMINAL             | `uint32` | 512    | Under-pressure warning                                                                 |
| <a href="#FLAG_FUEL_PRESSURE_ABOVE_NOMINAL"></a> FLAG_FUEL_PRESSURE_ABOVE_NOMINAL             | `uint32` | 1024   | Over-pressure warning                                                                  |
| <a href="#FLAG_DETONATION_SUPPORTED"></a> FLAG_DETONATION_SUPPORTED                                                                     | `uint32` | 2048   | Detonation warning. This flag is optional.             |
| <a href="#FLAG_DETONATION_OBSERVED"></a> FLAG_DETONATION_OBSERVED                                                                       | `uint32` | 4096   | Detonation condition observed warning                                                  |
| <a href="#FLAG_MISFIRE_SUPPORTED"></a> FLAG_MISFIRE_SUPPORTED                                                                           | `uint32` | 8192   | Misfire warning. This flag is optional.                |
| <a href="#FLAG_MISFIRE_OBSERVED"></a> FLAG_MISFIRE_OBSERVED                                                                             | `uint32` | 16384  | Misfire condition observed warning                                                     |
| <a href="#FLAG_OIL_PRESSURE_SUPPORTED"></a> FLAG_OIL_PRESSURE_SUPPORTED                                            | `uint32` | 32768  | Oil pressure. These flags are optional                                 |
| <a href="#FLAG_OIL_PRESSURE_BELOW_NOMINAL"></a> FLAG_OIL_PRESSURE_BELOW_NOMINAL               | `uint32` | 65536  | Under-pressure warning                                                                 |
| <a href="#FLAG_OIL_PRESSURE_ABOVE_NOMINAL"></a> FLAG_OIL_PRESSURE_ABOVE_NOMINAL               | `uint32` | 131072 | Over-pressure warning                                                                  |
| <a href="#FLAG_DEBRIS_SUPPORTED"></a> FLAG_DEBRIS_SUPPORTED                                                                             | `uint32` | 262144 | Debris warning. This flag is optional                                  |
| <a href="#FLAG_DEBRIS_DETECTED"></a> FLAG_DEBRIS_DETECTED                                                                               | `uint32` | 524288 | Detection of debris warning                                                            |
| <a href="#SPARK_PLUG_SINGLE"></a> SPARK_PLUG_SINGLE                                                                                     | `uint8`  | 0      |                                                                                        |
| <a href="#SPARK_PLUG_FIRST_ACTIVE"></a> SPARK_PLUG_FIRST_ACTIVE                                                    | `uint8`  | 1      |                                                                                        |
| <a href="#SPARK_PLUG_SECOND_ACTIVE"></a> SPARK_PLUG_SECOND_ACTIVE                                                  | `uint8`  | 2      |                                                                                        |
| <a href="#SPARK_PLUG_BOTH_ACTIVE"></a> SPARK_PLUG_BOTH_ACTIVE                                                      | `uint8`  | 3      |                                                                                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/InternalCombustionEngineStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp					# time since system start (microseconds)

uint8 STATE_STOPPED = 0					# The engine is not running. This is the default state.
uint8 STATE_STARTING = 1				# The engine is starting. This is a transient state.
uint8 STATE_RUNNING = 2					# The engine is running normally.
uint8 STATE_FAULT = 3					# The engine can no longer function.
uint8 state

uint32 FLAG_GENERAL_ERROR = 1				# General error.

uint32 FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED = 2	# Error of the crankshaft sensor. This flag is optional.
uint32 FLAG_CRANKSHAFT_SENSOR_ERROR = 4

uint32 FLAG_TEMPERATURE_SUPPORTED = 8			# Temperature levels. These flags are optional
uint32 FLAG_TEMPERATURE_BELOW_NOMINAL = 16      	# Under-temperature warning
uint32 FLAG_TEMPERATURE_ABOVE_NOMINAL = 32      	# Over-temperature warning
uint32 FLAG_TEMPERATURE_OVERHEATING = 64      		# Critical overheating
uint32 FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL = 128     	# Exhaust gas over-temperature warning

uint32 FLAG_FUEL_PRESSURE_SUPPORTED = 256		# Fuel pressure. These flags are optional
uint32 FLAG_FUEL_PRESSURE_BELOW_NOMINAL  = 512     	# Under-pressure warning
uint32 FLAG_FUEL_PRESSURE_ABOVE_NOMINAL = 1024   	# Over-pressure warning

uint32 FLAG_DETONATION_SUPPORTED = 2048			# Detonation warning. This flag is optional.
uint32 FLAG_DETONATION_OBSERVED = 4096    		# Detonation condition observed warning

uint32 FLAG_MISFIRE_SUPPORTED = 8192			# Misfire warning. This flag is optional.
uint32 FLAG_MISFIRE_OBSERVED = 16384   			# Misfire condition observed warning

uint32 FLAG_OIL_PRESSURE_SUPPORTED = 32768		# Oil pressure. These flags are optional
uint32 FLAG_OIL_PRESSURE_BELOW_NOMINAL = 65536   	# Under-pressure warning
uint32 FLAG_OIL_PRESSURE_ABOVE_NOMINAL = 131072  	# Over-pressure warning

uint32 FLAG_DEBRIS_SUPPORTED = 262144			# Debris warning. This flag is optional
uint32 FLAG_DEBRIS_DETECTED = 524288  			# Detection of debris warning
uint32 flags

uint8 engine_load_percent				# Engine load estimate, percent, [0, 127]
uint32 engine_speed_rpm					# Engine speed, revolutions per minute
float32 spark_dwell_time_ms 				# Spark dwell time, millisecond
float32 atmospheric_pressure_kpa			# Atmospheric (barometric) pressure, kilopascal
float32 intake_manifold_pressure_kpa			# Engine intake manifold pressure, kilopascal
float32 intake_manifold_temperature			# Engine intake manifold temperature, kelvin
float32 coolant_temperature				# Engine coolant temperature, kelvin
float32 oil_pressure					# Oil pressure, kilopascal
float32 oil_temperature					# Oil temperature, kelvin
float32 fuel_pressure					# Fuel pressure, kilopascal
float32 fuel_consumption_rate_cm3pm			# Instant fuel consumption estimate, (centimeter^3)/minute
float32 estimated_consumed_fuel_volume_cm3		# Estimate of the consumed fuel since the start of the engine, centimeter^3
uint8 throttle_position_percent				# Throttle position, percent
uint8 ecu_index						# The index of the publishing ECU


uint8 SPARK_PLUG_SINGLE         = 0
uint8 SPARK_PLUG_FIRST_ACTIVE   = 1
uint8 SPARK_PLUG_SECOND_ACTIVE  = 2
uint8 SPARK_PLUG_BOTH_ACTIVE    = 3
uint8 spark_plug_usage					# Spark plug activity report.

float32 ignition_timing_deg				# Cylinder ignition timing, angular degrees of the crankshaft
float32 injection_time_ms				# Fuel injection time, millisecond
float32 cylinder_head_temperature			# Cylinder head temperature (CHT), kelvin
float32 exhaust_gas_temperature				# Exhaust gas temperature (EGT), kelvin
float32 lambda_coefficient				# Estimated lambda coefficient, dimensionless ratio
```

:::
