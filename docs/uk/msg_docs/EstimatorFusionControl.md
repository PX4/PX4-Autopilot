---
pageClass: is-wide-page
---

# EstimatorFusionControl (UORB message)

**TOPICS:** estimator_fusion_control

## Fields

| Назва                                | Тип       | Unit [Frame] | Range/Enum | Опис                                                      |
| ------------------------------------ | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                            | `uint64`  |                                                                  |            | time since system start (microseconds) |
| gps_intended    | `bool[2]` |                                                                  |            |                                                           |
| of_intended     | `bool`    |                                                                  |            |                                                           |
| ev_intended     | `bool`    |                                                                  |            |                                                           |
| agp_intended    | `bool[4]` |                                                                  |            |                                                           |
| baro_intended   | `bool`    |                                                                  |            |                                                           |
| rng_intended    | `bool`    |                                                                  |            |                                                           |
| mag_intended    | `bool`    |                                                                  |            |                                                           |
| aspd_intended   | `bool`    |                                                                  |            |                                                           |
| rngbcn_intended | `bool`    |                                                                  |            |                                                           |
| gps_active      | `bool[2]` |                                                                  |            |                                                           |
| of_active       | `bool`    |                                                                  |            |                                                           |
| ev_active       | `bool`    |                                                                  |            |                                                           |
| agp_active      | `bool[4]` |                                                                  |            |                                                           |
| baro_active     | `bool`    |                                                                  |            |                                                           |
| rng_active      | `bool`    |                                                                  |            |                                                           |
| mag_active      | `bool`    |                                                                  |            |                                                           |
| aspd_active     | `bool`    |                                                                  |            |                                                           |
| rngbcn_active   | `bool`    |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorFusionControl.msg)

:::details
Click here to see original file

```c
uint64 timestamp                # time since system start (microseconds)

# sensor intended for fusion (enabled via EKF2_SENS_EN AND CTRL param != disabled)
bool[2] gps_intended
bool of_intended
bool ev_intended
bool[4] agp_intended
bool baro_intended
bool rng_intended
bool mag_intended
bool aspd_intended
bool rngbcn_intended

# whether the estimator is actively fusing data from each source
bool[2] gps_active
bool of_active
bool ev_active
bool[4] agp_active
bool baro_active
bool rng_active
bool mag_active
bool aspd_active
bool rngbcn_active
```

:::
