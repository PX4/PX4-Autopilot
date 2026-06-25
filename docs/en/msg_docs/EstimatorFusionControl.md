---
pageClass: is-wide-page
---

# EstimatorFusionControl (UORB message)

**TOPICS:** estimator_fusion_control

## Fields

| Name                                            | Type      | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------------------- | --------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp             | `uint64`  |              |            | time since system start (microseconds) |
| <a id="fld_gps_intended"></a>gps_intended       | `bool[2]` |              |            |
| <a id="fld_of_intended"></a>of_intended         | `bool`    |              |            |
| <a id="fld_ev_intended"></a>ev_intended         | `bool`    |              |            |
| <a id="fld_agp_intended"></a>agp_intended       | `bool[4]` |              |            |
| <a id="fld_baro_intended"></a>baro_intended     | `bool`    |              |            |
| <a id="fld_rng_intended"></a>rng_intended       | `bool`    |              |            |
| <a id="fld_mag_intended"></a>mag_intended       | `bool`    |              |            |
| <a id="fld_aspd_intended"></a>aspd_intended     | `bool`    |              |            |
| <a id="fld_rngbcn_intended"></a>rngbcn_intended | `bool`    |              |            |
| <a id="fld_gps_active"></a>gps_active           | `bool[2]` |              |            |
| <a id="fld_of_active"></a>of_active             | `bool`    |              |            |
| <a id="fld_ev_active"></a>ev_active             | `bool`    |              |            |
| <a id="fld_agp_active"></a>agp_active           | `bool[4]` |              |            |
| <a id="fld_baro_active"></a>baro_active         | `bool`    |              |            |
| <a id="fld_rng_active"></a>rng_active           | `bool`    |              |            |
| <a id="fld_mag_active"></a>mag_active           | `bool`    |              |            |
| <a id="fld_aspd_active"></a>aspd_active         | `bool`    |              |            |
| <a id="fld_rngbcn_active"></a>rngbcn_active     | `bool`    |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorFusionControl.msg)

::: details Click here to see original file

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
