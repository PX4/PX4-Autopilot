---
pageClass: is-wide-page
---

# EstimatorGpsStatus (UORB message)

**TOPICS:** estimator_gpsstatus

## Fields

| 명칭                                                                                                                                          | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                               |
| ------------------------------------------------------------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------ |
| timestamp                                                                                                                                   | `uint64`  |                                                                  |            | time since system start (microseconds)                                        |
| timestamp_sample                                                                                                       | `uint64`  |                                                                  |            | the timestamp of the raw data (microseconds)                                  |
| checks_passed                                                                                                          | `bool`    |                                                                  |            |                                                                                                  |
| check_fail_gps_fix                                                           | `bool`    |                                                                  |            | 0 : insufficient fix type (no 3D solution)                    |
| check_fail_min_sat_count                                | `bool`    |                                                                  |            | 1 : minimum required sat count fail                                              |
| check_fail_max_pdop                                                          | `bool`    |                                                                  |            | 2 : maximum allowed PDOP fail                                                    |
| check_fail_max_horz_err                                 | `bool`    |                                                                  |            | 3 : maximum allowed horizontal position error fail                               |
| check_fail_max_vert_err                                 | `bool`    |                                                                  |            | 4 : maximum allowed vertical position error fail                                 |
| check_fail_max_spd_err                                  | `bool`    |                                                                  |            | 5 : maximum allowed speed error fail                                             |
| check_fail_max_horz_drift                               | `bool`    |                                                                  |            | 6 : maximum allowed horizontal position drift fail - requires stationary vehicle |
| check_fail_max_vert_drift                               | `bool`    |                                                                  |            | 7 : maximum allowed vertical position drift fail - requires stationary vehicle   |
| check_fail_max_horz_spd_err        | `bool`    |                                                                  |            | 8 : maximum allowed horizontal speed fail - requires stationary vehicle          |
| check_fail_max_vert_spd_err        | `bool`    |                                                                  |            | 9 : maximum allowed vertical velocity discrepancy fail                           |
| check_fail_spoofed_gps                                                       | `bool`    |                                                                  |            | 10 : GPS signal is spoofed                                                       |
| position_drift_rate_horizontal_m_s | `float32` |                                                                  |            | Horizontal position rate magnitude (m/s)                                      |
| position_drift_rate_vertical_m_s   | `float32` |                                                                  |            | Vertical position rate magnitude (m/s)                                        |
| filtered_horizontal_speed_m_s                           | `float32` |                                                                  |            | Filtered horizontal velocity magnitude (m/s)                                  |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorGpsStatus.msg)

:::details
Click here to see original file

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
