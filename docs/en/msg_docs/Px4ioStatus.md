---
pageClass: is-wide-page
---

# Px4ioStatus (UORB message)

**TOPICS:** px4io_status

## Fields

| Name                                                                    | Type         | Unit [Frame] | Range/Enum | Description                                              |
| ----------------------------------------------------------------------- | ------------ | ------------ | ---------- | -------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                     | `uint64`     |              |            | time since system start (microseconds)                   |
| <a id="fld_free_memory_bytes"></a>free_memory_bytes                     | `uint16`     |              |            |
| <a id="fld_voltage_v"></a>voltage_v                                     | `float32`    |              |            | Servo rail voltage in volts                              |
| <a id="fld_rssi_v"></a>rssi_v                                           | `float32`    |              |            | RSSI pin voltage in volts                                |
| <a id="fld_status_arm_sync"></a>status_arm_sync                         | `bool`       |              |            |
| <a id="fld_status_failsafe"></a>status_failsafe                         | `bool`       |              |            |
| <a id="fld_status_fmu_initialized"></a>status_fmu_initialized           | `bool`       |              |            |
| <a id="fld_status_fmu_ok"></a>status_fmu_ok                             | `bool`       |              |            |
| <a id="fld_status_init_ok"></a>status_init_ok                           | `bool`       |              |            |
| <a id="fld_status_outputs_armed"></a>status_outputs_armed               | `bool`       |              |            |
| <a id="fld_status_raw_pwm"></a>status_raw_pwm                           | `bool`       |              |            |
| <a id="fld_status_rc_ok"></a>status_rc_ok                               | `bool`       |              |            |
| <a id="fld_status_rc_dsm"></a>status_rc_dsm                             | `bool`       |              |            |
| <a id="fld_status_rc_ppm"></a>status_rc_ppm                             | `bool`       |              |            |
| <a id="fld_status_rc_sbus"></a>status_rc_sbus                           | `bool`       |              |            |
| <a id="fld_status_rc_st24"></a>status_rc_st24                           | `bool`       |              |            |
| <a id="fld_status_rc_sumd"></a>status_rc_sumd                           | `bool`       |              |            |
| <a id="fld_status_safety_button_event"></a>status_safety_button_event   | `bool`       |              |            | px4io safety button was pressed for longer than 1 second |
| <a id="fld_alarm_pwm_error"></a>alarm_pwm_error                         | `bool`       |              |            |
| <a id="fld_alarm_rc_lost"></a>alarm_rc_lost                             | `bool`       |              |            |
| <a id="fld_arming_failsafe_custom"></a>arming_failsafe_custom           | `bool`       |              |            |
| <a id="fld_arming_fmu_armed"></a>arming_fmu_armed                       | `bool`       |              |            |
| <a id="fld_arming_fmu_prearmed"></a>arming_fmu_prearmed                 | `bool`       |              |            |
| <a id="fld_arming_termination"></a>arming_termination                   | `bool`       |              |            |
| <a id="fld_arming_io_arm_ok"></a>arming_io_arm_ok                       | `bool`       |              |            |
| <a id="fld_arming_lockdown"></a>arming_lockdown                         | `bool`       |              |            |
| <a id="fld_arming_termination_failsafe"></a>arming_termination_failsafe | `bool`       |              |            |
| <a id="fld_pwm"></a>pwm                                                 | `uint16[8]`  |              |            |
| <a id="fld_pwm_disarmed"></a>pwm_disarmed                               | `uint16[8]`  |              |            |
| <a id="fld_pwm_failsafe"></a>pwm_failsafe                               | `uint16[8]`  |              |            |
| <a id="fld_pwm_rate_hz"></a>pwm_rate_hz                                 | `uint16[8]`  |              |            |
| <a id="fld_raw_inputs"></a>raw_inputs                                   | `uint16[18]` |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Px4ioStatus.msg)

::: details Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint16 free_memory_bytes

float32 voltage_v		# Servo rail voltage in volts
float32 rssi_v			# RSSI pin voltage in volts

# PX4IO status flags (PX4IO_P_STATUS_FLAGS)
bool status_arm_sync
bool status_failsafe
bool status_fmu_initialized
bool status_fmu_ok
bool status_init_ok
bool status_outputs_armed
bool status_raw_pwm
bool status_rc_ok
bool status_rc_dsm
bool status_rc_ppm
bool status_rc_sbus
bool status_rc_st24
bool status_rc_sumd
bool status_safety_button_event # px4io safety button was pressed for longer than 1 second

# PX4IO alarms (PX4IO_P_STATUS_ALARMS)
bool alarm_pwm_error
bool alarm_rc_lost

# PX4IO arming (PX4IO_P_SETUP_ARMING)
bool arming_failsafe_custom
bool arming_fmu_armed
bool arming_fmu_prearmed
bool arming_termination
bool arming_io_arm_ok
bool arming_lockdown
bool arming_termination_failsafe

uint16[8] pwm
uint16[8] pwm_disarmed
uint16[8] pwm_failsafe

uint16[8] pwm_rate_hz

uint16[18] raw_inputs
```

:::
