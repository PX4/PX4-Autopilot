---
pageClass: is-wide-page
---

# TecsStatus (UORB message)

**TOPICS:** tecs_status

## Fields

| Name                                                                      | Type      | Unit [Frame] | Range/Enum | Description                                                                                                          |
| ------------------------------------------------------------------------- | --------- | ------------ | ---------- | -------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                       | `uint64`  |              |            | time since system start (microseconds)                                                                               |
| <a id="fld_altitude_sp"></a>altitude_sp                                   | `float32` |              |            | Altitude setpoint AMSL [m]                                                                                           |
| <a id="fld_altitude_reference"></a>altitude_reference                     | `float32` |              |            | Altitude setpoint reference AMSL [m]                                                                                 |
| <a id="fld_altitude_time_constant"></a>altitude_time_constant             | `float32` |              |            | Time constant of the altitude tracker [s]                                                                            |
| <a id="fld_height_rate_reference"></a>height_rate_reference               | `float32` |              |            | Height rate setpoint reference [m/s]                                                                                 |
| <a id="fld_height_rate_direct"></a>height_rate_direct                     | `float32` |              |            | Direct height rate setpoint from velocity reference generator [m/s]                                                  |
| <a id="fld_height_rate_setpoint"></a>height_rate_setpoint                 | `float32` |              |            | Height rate setpoint [m/s]                                                                                           |
| <a id="fld_height_rate"></a>height_rate                                   | `float32` |              |            | Height rate [m/s]                                                                                                    |
| <a id="fld_equivalent_airspeed_sp"></a>equivalent_airspeed_sp             | `float32` |              |            | Equivalent airspeed setpoint [m/s]                                                                                   |
| <a id="fld_true_airspeed_sp"></a>true_airspeed_sp                         | `float32` |              |            | True airspeed setpoint [m/s]                                                                                         |
| <a id="fld_true_airspeed_filtered"></a>true_airspeed_filtered             | `float32` |              |            | True airspeed filtered [m/s]                                                                                         |
| <a id="fld_true_airspeed_derivative_sp"></a>true_airspeed_derivative_sp   | `float32` |              |            | True airspeed derivative setpoint [m/s^2]                                                                            |
| <a id="fld_true_airspeed_derivative"></a>true_airspeed_derivative         | `float32` |              |            | True airspeed derivative [m/s^2]                                                                                     |
| <a id="fld_true_airspeed_derivative_raw"></a>true_airspeed_derivative_raw | `float32` |              |            | True airspeed derivative raw [m/s^2]                                                                                 |
| <a id="fld_total_energy_rate_sp"></a>total_energy_rate_sp                 | `float32` |              |            | Total energy rate setpoint [m^2/s^3]                                                                                 |
| <a id="fld_total_energy_rate"></a>total_energy_rate                       | `float32` |              |            | Total energy rate estimate [m^2/s^3]                                                                                 |
| <a id="fld_total_energy_balance_rate_sp"></a>total_energy_balance_rate_sp | `float32` |              |            | Energy balance rate setpoint [m^2/s^3]                                                                               |
| <a id="fld_total_energy_balance_rate"></a>total_energy_balance_rate       | `float32` |              |            | Energy balance rate estimate [m^2/s^3]                                                                               |
| <a id="fld_throttle_integ"></a>throttle_integ                             | `float32` |              |            | Throttle integrator value [-]                                                                                        |
| <a id="fld_pitch_integ"></a>pitch_integ                                   | `float32` |              |            | Pitch integrator value [rad]                                                                                         |
| <a id="fld_throttle_sp"></a>throttle_sp                                   | `float32` |              |            | Current throttle setpoint [-]                                                                                        |
| <a id="fld_pitch_sp_rad"></a>pitch_sp_rad                                 | `float32` |              |            | Current pitch setpoint [rad]                                                                                         |
| <a id="fld_throttle_trim"></a>throttle_trim                               | `float32` |              |            | estimated throttle value [0,1] required to fly level at equivalent_airspeed_sp in the current atmospheric conditions |
| <a id="fld_underspeed_ratio"></a>underspeed_ratio                         | `float32` |              |            | 0: no underspeed, 1: maximal underspeed. Controller takes measures to avoid stall proportional to ratio if >0.       |
| <a id="fld_fast_descend_ratio"></a>fast_descend_ratio                     | `float32` |              |            | value indicating if fast descend mode is enabled with ramp up and ramp down [0-1]                                    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TecsStatus.msg)

::: details Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

float32 altitude_sp			# Altitude setpoint AMSL [m]
float32 altitude_reference		# Altitude setpoint reference AMSL [m]
float32 altitude_time_constant		# Time constant of the altitude tracker [s]
float32 height_rate_reference		# Height rate setpoint reference [m/s]
float32 height_rate_direct		# Direct height rate setpoint from velocity reference generator [m/s]
float32 height_rate_setpoint		# Height rate setpoint [m/s]
float32 height_rate			# Height rate [m/s]
float32 equivalent_airspeed_sp		# Equivalent airspeed setpoint [m/s]
float32 true_airspeed_sp		# True airspeed setpoint [m/s]
float32 true_airspeed_filtered		# True airspeed filtered [m/s]
float32 true_airspeed_derivative_sp	# True airspeed derivative setpoint [m/s^2]
float32 true_airspeed_derivative	# True airspeed derivative [m/s^2]
float32 true_airspeed_derivative_raw	# True airspeed derivative raw [m/s^2]

float32 total_energy_rate_sp		# Total energy rate setpoint [m^2/s^3]
float32 total_energy_rate		# Total energy rate estimate [m^2/s^3]

float32 total_energy_balance_rate_sp	# Energy balance rate setpoint [m^2/s^3]
float32 total_energy_balance_rate	# Energy balance rate estimate [m^2/s^3]

float32 throttle_integ			# Throttle integrator value [-]
float32 pitch_integ			# Pitch integrator value [rad]

float32 throttle_sp			# Current throttle setpoint [-]
float32 pitch_sp_rad			# Current pitch setpoint [rad]
float32 throttle_trim			# estimated throttle value [0,1] required to fly level at equivalent_airspeed_sp in the current atmospheric conditions

float32 underspeed_ratio		# 0: no underspeed, 1: maximal underspeed. Controller takes measures to avoid stall proportional to ratio if >0.
float32 fast_descend_ratio 		#  value indicating if fast descend mode is enabled with ramp up and ramp down [0-1]
```

:::
