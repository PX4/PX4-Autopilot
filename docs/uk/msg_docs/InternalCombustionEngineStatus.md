---
pageClass: is-wide-page
---

# InternalCombustionEngineStatus (повідомлення UORB)

**TOPICS:** internal_combustion_engine_status

## Fields

| Назва                                                                                                                                                                     | Тип       | Unit [Frame] | Range/Enum                                                                       | Опис                                                        |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | -------------------------------------------------------------------------------- | ----------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                                                                                                       | `uint64`  | us                                                               |                                                                                  | Time since system start                                     |
| <a id="fld_state"></a>state                                                                                                                                               | `uint8`   |                                                                  | [STATE](#STATE)                                                                  | Engine state                                                |
| <a id="fld_substate"></a>substate                                                                                                                                         | `uint8`   |                                                                  | [SUBSTATE](#SUBSTATE)                                                            | Engine substate                                             |
| <a id="fld_flags"></a>flags                                                                                                                                               | `uint32`  |                                                                  | [FLAG](#FLAG)                                                                    | Engine status flags bitmap                                  |
| <a id="fld_engine_load_percent"></a>engine_load_percent                                                                         | `uint8`   | %                                                                | [0 : 127]    | Engine load estimate                                        |
| <a id="fld_engine_speed_rpm"></a>engine_speed_rpm                                                                               | `uint32`  | rpm                                                              | [0 : inf]    | Engine speed                                                |
| <a id="fld_spark_dwell_time_ms"></a>spark_dwell_time_ms                                                    | `float32` | ms                                                               | [0 : inf]    | Spark dwell time                                            |
| <a id="fld_atmospheric_pressure_kpa"></a>atmospheric_pressure_kpa                                                               | `float32` | kPa                                                              | [0 : inf]    | Atmospheric (barometric) pressure        |
| <a id="fld_intake_manifold_pressure_kpa"></a>intake_manifold_pressure_kpa                                  | `float32` | kPa                                                              | [0 : inf]    | Engine intake manifold pressure                             |
| <a id="fld_intake_manifold_temperature"></a>intake_manifold_temperature                                                         | `float32` | K                                                                | [0 : inf]    | Engine intake manifold temperature                          |
| <a id="fld_coolant_temperature"></a>coolant_temperature                                                                                              | `float32` | K                                                                | [0 : inf]    | Engine coolant temperature                                  |
| <a id="fld_oil_pressure"></a>oil_pressure                                                                                                            | `float32` | kPa                                                              | [0 : inf]    | Oil pressure                                                |
| <a id="fld_oil_temperature"></a>oil_temperature                                                                                                      | `float32` | K                                                                | [0 : inf]    | Oil temperature                                             |
| <a id="fld_fuel_pressure"></a>fuel_pressure                                                                                                          | `float32` | kPa                                                              | [0 : inf]    | Fuel pressure                                               |
| <a id="fld_fuel_consumption_rate_cm3pm"></a>fuel_consumption_rate_cm3pm                                    | `float32` | cm^3/min                                                         | [0 : inf]    | Instant fuel consumption estimate                           |
| <a id="fld_estimated_consumed_fuel_volume_cm3"></a>estimated_consumed_fuel_volume_cm3 | `float32` | cm^3                                                             | [0 : inf]    | Estimate of the consumed fuel since the start of the engine |
| <a id="fld_throttle_position_percent"></a>throttle_position_percent                                                             | `uint8`   | %                                                                | [0 : 100]    | Throttle position                                           |
| <a id="fld_ecu_index"></a>ecu_index                                                                                                                  | `uint8`   |                                                                  | [0 : 255]    | The index of the publishing ECU                             |
| <a id="fld_spark_plug_usage"></a>spark_plug_usage                                                                               | `uint8`   |                                                                  | [SPARK_PLUG](#SPARK_PLUG)                                   | Spark plug activity report                                  |
| <a id="fld_ignition_timing_deg"></a>ignition_timing_deg                                                                         | `float32` | deg                                                              | [-inf : inf] | Cylinder ignition timing, angular degrees of the crankshaft |
| <a id="fld_injection_time_ms"></a>injection_time_ms                                                                             | `float32` | ms                                                               | [0 : inf]    | Fuel injection time                                         |
| <a id="fld_cylinder_head_temperature"></a>cylinder_head_temperature                                                             | `float32` | K                                                                | [0 : inf]    | Cylinder head temperature (CHT)          |
| <a id="fld_exhaust_gas_temperature"></a>exhaust_gas_temperature                                                                 | `float32` | K                                                                | [0 : inf]    | Exhaust gas temperature (EGT)            |
| <a id="fld_lambda_coefficient"></a>lambda_coefficient                                                                                                | `float32` |                                                                  | [0 : inf]    | Estimated lambda coefficient, dimensionless ratio           |
| <a id="fld_pid_idle_rpm_integral"></a>pid_idle_rpm_integral                                                | `float32` |                                                                  | [-1 : 1]     | Integral of the PID for the closed loop idle RPM controller |

## Enums

### STATE {#STATE}

Used in field(s): [state](#fld_state)

| Назва                                                            | Тип     | Значення | Опис                                                                                  |
| ---------------------------------------------------------------- | ------- | -------- | ------------------------------------------------------------------------------------- |
| <a id="#STATE_STOPPED"></a> STATE_STOPPED   | `uint8` | 0        | The engine is not running. This is the default state. |
| <a id="#STATE_STARTING"></a> STATE_STARTING | `uint8` | 1        | The engine is starting. This is a transient state.    |
| <a id="#STATE_RUNNING"></a> STATE_RUNNING   | `uint8` | 2        | The engine is running normally.                                       |
| <a id="#STATE_FAULT"></a> STATE_FAULT       | `uint8` | 3        | The engine can no longer function.                                    |

### SUBSTATE {#SUBSTATE}

Used in field(s): [substate](#fld_substate)

| Назва                                                          | Тип     | Значення | Опис                                                                              |
| -------------------------------------------------------------- | ------- | -------- | --------------------------------------------------------------------------------- |
| <a id="#SUBSTATE_RUN"></a> SUBSTATE_RUN   | `uint8` | 0        | The engine is running. This is the default state. |
| <a id="#SUBSTATE_IDLE"></a> SUBSTATE_IDLE | `uint8` | 1        | The engine idle rpm controller is running.                        |
| <a id="#SUBSTATE_REST"></a> SUBSTATE_REST | `uint8` | 2        | The engine is at rest.                                            |

### FLAG {#FLAG}

Used in field(s): [flags](#fld_flags)

| Назва                                                                                                                                                                           | Тип      | Значення | Опис                                                                                   |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | -------- | -------------------------------------------------------------------------------------- |
| <a id="#FLAG_GENERAL_ERROR"></a> FLAG_GENERAL_ERROR                                                                                   | `uint32` | 1        | General error.                                                         |
| <a id="#FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED"></a> FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED | `uint32` | 2        | Error of the crankshaft sensor. This flag is optional. |
| <a id="#FLAG_CRANKSHAFT_SENSOR_ERROR"></a> FLAG_CRANKSHAFT_SENSOR_ERROR                                          | `uint32` | 4        |                                                                                        |
| <a id="#FLAG_TEMPERATURE_SUPPORTED"></a> FLAG_TEMPERATURE_SUPPORTED                                                                   | `uint32` | 8        | Temperature levels. These flags are optional                           |
| <a id="#FLAG_TEMPERATURE_BELOW_NOMINAL"></a> FLAG_TEMPERATURE_BELOW_NOMINAL                                      | `uint32` | 16       | Under-temperature warning                                                              |
| <a id="#FLAG_TEMPERATURE_ABOVE_NOMINAL"></a> FLAG_TEMPERATURE_ABOVE_NOMINAL                                      | `uint32` | 32       | Over-temperature warning                                                               |
| <a id="#FLAG_TEMPERATURE_OVERHEATING"></a> FLAG_TEMPERATURE_OVERHEATING                                                               | `uint32` | 64       | Critical overheating                                                                   |
| <a id="#FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL"></a> FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL         | `uint32` | 128      | Exhaust gas over-temperature warning                                                   |
| <a id="#FLAG_FUEL_PRESSURE_SUPPORTED"></a> FLAG_FUEL_PRESSURE_SUPPORTED                                          | `uint32` | 256      | Fuel pressure. These flags are optional                                |
| <a id="#FLAG_FUEL_PRESSURE_BELOW_NOMINAL"></a> FLAG_FUEL_PRESSURE_BELOW_NOMINAL             | `uint32` | 512      | Under-pressure warning                                                                 |
| <a id="#FLAG_FUEL_PRESSURE_ABOVE_NOMINAL"></a> FLAG_FUEL_PRESSURE_ABOVE_NOMINAL             | `uint32` | 1024     | Over-pressure warning                                                                  |
| <a id="#FLAG_DETONATION_SUPPORTED"></a> FLAG_DETONATION_SUPPORTED                                                                     | `uint32` | 2048     | Detonation warning. This flag is optional.             |
| <a id="#FLAG_DETONATION_OBSERVED"></a> FLAG_DETONATION_OBSERVED                                                                       | `uint32` | 4096     | Detonation condition observed warning                                                  |
| <a id="#FLAG_MISFIRE_SUPPORTED"></a> FLAG_MISFIRE_SUPPORTED                                                                           | `uint32` | 8192     | Misfire warning. This flag is optional.                |
| <a id="#FLAG_MISFIRE_OBSERVED"></a> FLAG_MISFIRE_OBSERVED                                                                             | `uint32` | 16384    | Misfire condition observed warning                                                     |
| <a id="#FLAG_OIL_PRESSURE_SUPPORTED"></a> FLAG_OIL_PRESSURE_SUPPORTED                                            | `uint32` | 32768    | Oil pressure. These flags are optional                                 |
| <a id="#FLAG_OIL_PRESSURE_BELOW_NOMINAL"></a> FLAG_OIL_PRESSURE_BELOW_NOMINAL               | `uint32` | 65536    | Under-pressure warning                                                                 |
| <a id="#FLAG_OIL_PRESSURE_ABOVE_NOMINAL"></a> FLAG_OIL_PRESSURE_ABOVE_NOMINAL               | `uint32` | 131072   | Over-pressure warning                                                                  |
| <a id="#FLAG_DEBRIS_SUPPORTED"></a> FLAG_DEBRIS_SUPPORTED                                                                             | `uint32` | 262144   | Debris warning. This flag is optional                                  |
| <a id="#FLAG_DEBRIS_DETECTED"></a> FLAG_DEBRIS_DETECTED                                                                               | `uint32` | 524288   | Detection of debris warning                                                            |

### SPARK_PLUG {#SPARK_PLUG}

Used in field(s): [spark_plug_usage](#fld_spark_plug_usage)

| Назва                                                                                                                          | Тип     | Значення | Опис |
| ------------------------------------------------------------------------------------------------------------------------------ | ------- | -------- | ---- |
| <a id="#SPARK_PLUG_SINGLE"></a> SPARK_PLUG_SINGLE                                    | `uint8` | 0        |      |
| <a id="#SPARK_PLUG_FIRST_ACTIVE"></a> SPARK_PLUG_FIRST_ACTIVE   | `uint8` | 1        |      |
| <a id="#SPARK_PLUG_SECOND_ACTIVE"></a> SPARK_PLUG_SECOND_ACTIVE | `uint8` | 2        |      |
| <a id="#SPARK_PLUG_BOTH_ACTIVE"></a> SPARK_PLUG_BOTH_ACTIVE     | `uint8` | 3        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/InternalCombustionEngineStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp					# [us] Time since system start

uint8 STATE_STOPPED = 0					# The engine is not running. This is the default state.
uint8 STATE_STARTING = 1				# The engine is starting. This is a transient state.
uint8 STATE_RUNNING = 2					# The engine is running normally.
uint8 STATE_FAULT = 3					# The engine can no longer function.
uint8 state						# [@enum STATE] Engine state

uint8 SUBSTATE_RUN = 0					# The engine is running. This is the default state.
uint8 SUBSTATE_IDLE = 1					# The engine idle rpm controller is running.
uint8 SUBSTATE_REST = 2					# The engine is at rest.
uint8 substate						# [@enum SUBSTATE] Engine substate

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
uint32 flags						# [@enum FLAG] Engine status flags bitmap

uint8 engine_load_percent				# [%] [@range 0, 127] Engine load estimate
uint32 engine_speed_rpm					# [rpm] [@range 0, inf] Engine speed
float32 spark_dwell_time_ms 				# [ms] [@range 0, inf] Spark dwell time
float32 atmospheric_pressure_kpa			# [kPa] [@range 0, inf] Atmospheric (barometric) pressure
float32 intake_manifold_pressure_kpa			# [kPa] [@range 0, inf] Engine intake manifold pressure
float32 intake_manifold_temperature			# [K] [@range 0, inf] Engine intake manifold temperature
float32 coolant_temperature				# [K] [@range 0, inf] Engine coolant temperature
float32 oil_pressure					# [kPa] [@range 0, inf] Oil pressure
float32 oil_temperature					# [K] [@range 0, inf] Oil temperature
float32 fuel_pressure					# [kPa] [@range 0, inf] Fuel pressure
float32 fuel_consumption_rate_cm3pm			# [cm^3/min] [@range 0, inf] Instant fuel consumption estimate
float32 estimated_consumed_fuel_volume_cm3		# [cm^3] [@range 0, inf] Estimate of the consumed fuel since the start of the engine
uint8 throttle_position_percent				# [%] [@range 0, 100] Throttle position
uint8 ecu_index						# [-] [@range 0, 255] The index of the publishing ECU


uint8 SPARK_PLUG_SINGLE         = 0
uint8 SPARK_PLUG_FIRST_ACTIVE   = 1
uint8 SPARK_PLUG_SECOND_ACTIVE  = 2
uint8 SPARK_PLUG_BOTH_ACTIVE    = 3
uint8 spark_plug_usage					# [@enum SPARK_PLUG] Spark plug activity report

float32 ignition_timing_deg				# [deg] [@range -inf, inf] Cylinder ignition timing, angular degrees of the crankshaft
float32 injection_time_ms				# [ms] [@range 0, inf] Fuel injection time
float32 cylinder_head_temperature			# [K] [@range 0, inf] Cylinder head temperature (CHT)
float32 exhaust_gas_temperature				# [K] [@range 0, inf] Exhaust gas temperature (EGT)
float32 lambda_coefficient				# [-] [@range 0, inf] Estimated lambda coefficient, dimensionless ratio

float32 pid_idle_rpm_integral				# [-] [@range -1, 1] Integral of the PID for the closed loop idle RPM controller
```

:::
