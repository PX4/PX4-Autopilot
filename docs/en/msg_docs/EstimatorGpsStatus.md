---
pageClass: is-wide-page
---

# EstimatorGpsStatus (UORB message)

**TOPICS:** estimator_gps_status

## Fields

| Name                                                                                  | Type      | Unit [Frame] | Range/Enum | Description                                                                      |
| ------------------------------------------------------------------------------------- | --------- | ------------ | ---------- | -------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                   | `uint64`  |              |            | time since system start (microseconds)                                           |
| <a id="fld_timestamp_sample"></a>timestamp_sample                                     | `uint64`  |              |            | the timestamp of the raw data (microseconds)                                     |
| <a id="fld_checks_passed"></a>checks_passed                                           | `bool`    |              |            |
| <a id="fld_check_fail_gps_fix"></a>check_fail_gps_fix                                 | `bool`    |              |            | 0 : insufficient fix type (no 3D solution)                                       |
| <a id="fld_check_fail_min_sat_count"></a>check_fail_min_sat_count                     | `bool`    |              |            | 1 : minimum required sat count fail                                              |
| <a id="fld_check_fail_max_pdop"></a>check_fail_max_pdop                               | `bool`    |              |            | 2 : maximum allowed PDOP fail                                                    |
| <a id="fld_check_fail_max_horz_err"></a>check_fail_max_horz_err                       | `bool`    |              |            | 3 : maximum allowed horizontal position error fail                               |
| <a id="fld_check_fail_max_vert_err"></a>check_fail_max_vert_err                       | `bool`    |              |            | 4 : maximum allowed vertical position error fail                                 |
| <a id="fld_check_fail_max_spd_err"></a>check_fail_max_spd_err                         | `bool`    |              |            | 5 : maximum allowed speed error fail                                             |
| <a id="fld_check_fail_max_horz_drift"></a>check_fail_max_horz_drift                   | `bool`    |              |            | 6 : maximum allowed horizontal position drift fail - requires stationary vehicle |
| <a id="fld_check_fail_max_vert_drift"></a>check_fail_max_vert_drift                   | `bool`    |              |            | 7 : maximum allowed vertical position drift fail - requires stationary vehicle   |
| <a id="fld_check_fail_max_horz_spd_err"></a>check_fail_max_horz_spd_err               | `bool`    |              |            | 8 : maximum allowed horizontal speed fail - requires stationary vehicle          |
| <a id="fld_check_fail_max_vert_spd_err"></a>check_fail_max_vert_spd_err               | `bool`    |              |            | 9 : maximum allowed vertical velocity discrepancy fail                           |
| <a id="fld_check_fail_spoofed_gps"></a>check_fail_spoofed_gps                         | `bool`    |              |            | 10 : GPS signal is spoofed                                                       |
| <a id="fld_position_drift_rate_horizontal_m_s"></a>position_drift_rate_horizontal_m_s | `float32` |              |            | Horizontal position rate magnitude (m/s)                                         |
| <a id="fld_position_drift_rate_vertical_m_s"></a>position_drift_rate_vertical_m_s     | `float32` |              |            | Vertical position rate magnitude (m/s)                                           |
| <a id="fld_filtered_horizontal_speed_m_s"></a>filtered_horizontal_speed_m_s           | `float32` |              |            | Filtered horizontal velocity magnitude (m/s)                                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorGpsStatus.msg)

::: details Click here to see original file

```c
uint64 timestamp                          # time since system start (microseconds)
uint64 timestamp_sample                   # the timestamp of the raw data (microseconds)

bool checks_passed

bool check_fail_gps_fix          # 0 : insufficient fix type (no 3D solution)
bool check_fail_min_sat_count    # 1 : minimum required sat count fail
bool check_fail_max_pdop         # 2 : maximum allowed PDOP fail
bool check_fail_max_horz_err     # 3 : maximum allowed horizontal position error fail
bool check_fail_max_vert_err     # 4 : maximum allowed vertical position error fail
bool check_fail_max_spd_err      # 5 : maximum allowed speed error fail
bool check_fail_max_horz_drift   # 6 : maximum allowed horizontal position drift fail - requires stationary vehicle
bool check_fail_max_vert_drift   # 7 : maximum allowed vertical position drift fail - requires stationary vehicle
bool check_fail_max_horz_spd_err # 8 : maximum allowed horizontal speed fail - requires stationary vehicle
bool check_fail_max_vert_spd_err # 9 : maximum allowed vertical velocity discrepancy fail
bool check_fail_spoofed_gps      # 10 : GPS signal is spoofed

float32 position_drift_rate_horizontal_m_s # Horizontal position rate magnitude (m/s)
float32 position_drift_rate_vertical_m_s   # Vertical position rate magnitude (m/s)
float32 filtered_horizontal_speed_m_s      # Filtered horizontal velocity magnitude (m/s)
```

:::
