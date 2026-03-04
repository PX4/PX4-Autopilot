---
pageClass: is-wide-page
---

# AirspeedValidatedV0 (UORB message)

**TOPICS:** airspeed_validatedv0

## Fields

| 参数名                                                                                                                                       | 类型        | Unit [Frame] | Range/Enum | 描述                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| timestamp                                                                                                                                 | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                            |
| indicated_airspeed_m_s                                                     | `float32` |                                                                  |            | indicated airspeed in m/s (IAS), set to NAN if invalid                                                            |
| calibrated_airspeed_m_s                                                    | `float32` |                                                                  |            | calibrated airspeed in m/s (CAS, accounts for instrumentation errors), set to NAN if invalid                      |
| true_airspeed_m_s                                                          | `float32` |                                                                  |            | true filtered airspeed in m/s (TAS), set to NAN if invalid                                                        |
| calibrated_ground_minus_wind_m_s | `float32` |                                                                  |            | CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid |
| true_ground_minus_wind_m_s       | `float32` |                                                                  |            | TAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid |
| airspeed_sensor_measurement_valid                                          | `bool`    |                                                                  |            | True if data from at least one airspeed sensor is declared valid.                                                    |
| selected_airspeed_index                                                                         | `int8`    |                                                                  |            | 1-3: airspeed sensor index, 0: groundspeed-windspeed, -1: airspeed invalid           |
| airspeed_derivative_filtered                                                                    | `float32` |                                                                  |            | filtered indicated airspeed derivative [m/s/s]                                   |
| throttle_filtered                                                                                                    | `float32` |                                                                  |            | filtered fixed-wing throttle [-]                                                 |
| pitch_filtered                                                                                                       | `float32` |                                                                  |            | filtered pitch [rad]                                                             |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述 |
| -------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/AirspeedValidatedV0.msg)

:::details
Click here to see original file

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp				# time since system start (microseconds)

float32 indicated_airspeed_m_s			# indicated airspeed in m/s (IAS), set to NAN if invalid
float32 calibrated_airspeed_m_s     		# calibrated airspeed in m/s (CAS, accounts for instrumentation errors), set to NAN if invalid
float32 true_airspeed_m_s			# true filtered airspeed in m/s (TAS), set to NAN if invalid

float32 calibrated_ground_minus_wind_m_s 	# CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid
float32 true_ground_minus_wind_m_s 		# TAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid

bool airspeed_sensor_measurement_valid 		# True if data from at least one airspeed sensor is declared valid.

int8 selected_airspeed_index 			# 1-3: airspeed sensor index, 0: groundspeed-windspeed, -1: airspeed invalid

float32 airspeed_derivative_filtered		# filtered indicated airspeed derivative [m/s/s]
float32 throttle_filtered			# filtered fixed-wing throttle [-]
float32 pitch_filtered				# filtered pitch [rad]
```

:::
