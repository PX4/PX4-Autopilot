---
pageClass: is-wide-page
---

# MagWorkerData (UORB message)

**TOPICS:** mag_worker_data

## Fields

| Name                                                                            | Type         | Unit [Frame] | Range/Enum | Description                            |
| ------------------------------------------------------------------------------- | ------------ | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                             | `uint64`     |              |            | time since system start (microseconds) |
| <a id="fld_timestamp_sample"></a>timestamp_sample                               | `uint64`     |              |            |
| <a id="fld_done_count"></a>done_count                                           | `uint32`     |              |            |
| <a id="fld_calibration_points_perside"></a>calibration_points_perside           | `uint32`     |              |            |
| <a id="fld_calibration_interval_perside_us"></a>calibration_interval_perside_us | `uint64`     |              |            |
| <a id="fld_calibration_counter_total"></a>calibration_counter_total             | `uint32[4]`  |              |            |
| <a id="fld_side_data_collected"></a>side_data_collected                         | `bool[4]`    |              |            |
| <a id="fld_x"></a>x                                                             | `float32[4]` |              |            |
| <a id="fld_y"></a>y                                                             | `float32[4]` |              |            |
| <a id="fld_z"></a>z                                                             | `float32[4]` |              |            |

## Constants

| Name                            | Type    | Value | Description |
| ------------------------------- | ------- | ----- | ----------- |
| <a id="#MAX_MAGS"></a> MAX_MAGS | `uint8` | 4     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MagWorkerData.msg)

::: details Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint8 MAX_MAGS = 4

uint32 done_count
uint32 calibration_points_perside
uint64 calibration_interval_perside_us
uint32[4] calibration_counter_total
bool[4] side_data_collected
float32[4] x
float32[4] y
float32[4] z
```

:::
