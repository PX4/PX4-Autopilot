---
pageClass: is-wide-page
---

# AirspeedValidatedV0 (UORB message)

**TOPICS:** airspeed_validated_v0

## Fields

| Name                                                                                | Type      | Unit [Frame] | Range/Enum | Description                                                                                                                          |
| ----------------------------------------------------------------------------------- | --------- | ------------ | ---------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                                                 | `uint64`  |              |            | time since system start (microseconds)                                                                                               |
| <a id="fld_indicated_airspeed_m_s"></a>indicated_airspeed_m_s                       | `float32` |              |            | indicated airspeed in m/s (IAS), set to NAN if invalid                                                                               |
| <a id="fld_calibrated_airspeed_m_s"></a>calibrated_airspeed_m_s                     | `float32` |              |            | calibrated airspeed in m/s (CAS, accounts for instrumentation errors), set to NAN if invalid                                         |
| <a id="fld_true_airspeed_m_s"></a>true_airspeed_m_s                                 | `float32` |              |            | true filtered airspeed in m/s (TAS), set to NAN if invalid                                                                           |
| <a id="fld_calibrated_ground_minus_wind_m_s"></a>calibrated_ground_minus_wind_m_s   | `float32` |              |            | CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid |
| <a id="fld_true_ground_minus_wind_m_s"></a>true_ground_minus_wind_m_s               | `float32` |              |            | TAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid |
| <a id="fld_airspeed_sensor_measurement_valid"></a>airspeed_sensor_measurement_valid | `bool`    |              |            | True if data from at least one airspeed sensor is declared valid.                                                                    |
| <a id="fld_selected_airspeed_index"></a>selected_airspeed_index                     | `int8`    |              |            | 1-3: airspeed sensor index, 0: groundspeed-windspeed, -1: airspeed invalid                                                           |
| <a id="fld_airspeed_derivative_filtered"></a>airspeed_derivative_filtered           | `float32` |              |            | filtered indicated airspeed derivative [m/s/s]                                                                                       |
| <a id="fld_throttle_filtered"></a>throttle_filtered                                 | `float32` |              |            | filtered fixed-wing throttle [-]                                                                                                     |
| <a id="fld_pitch_filtered"></a>pitch_filtered                                       | `float32` |              |            | filtered pitch [rad]                                                                                                                 |

## Constants

| Name                                          | Type     | Value | Description |
| --------------------------------------------- | -------- | ----- | ----------- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/AirspeedValidatedV0.msg)

::: details Click here to see original file

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
