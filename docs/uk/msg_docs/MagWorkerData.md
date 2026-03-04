---
pageClass: is-wide-page
---

# MagWorkerData (UORB message)

**TOPICS:** mag_workerdata

## Fields

| Назва                                                                                          | Тип          | Unit [Frame] | Range/Enum | Опис                                                      |
| ---------------------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                                                      | `uint64`     |                                                                  |            | time since system start (microseconds) |
| timestamp_sample                                                          | `uint64`     |                                                                  |            |                                                           |
| done_count                                                                | `uint32`     |                                                                  |            |                                                           |
| calibration_points_perside                           | `uint32`     |                                                                  |            |                                                           |
| calibration_interval_perside_us | `uint64`     |                                                                  |            |                                                           |
| calibration_counter_total                            | `uint32[4]`  |                                                                  |            |                                                           |
| side_data_collected                                  | `bool[4]`    |                                                                  |            |                                                           |
| x                                                                                              | `float32[4]` |                                                                  |            |                                                           |
| y                                                                                              | `float32[4]` |                                                                  |            |                                                           |
| z                                                                                              | `float32[4]` |                                                                  |            |                                                           |

## Constants

| Назва                                                  | Тип     | Значення | Опис |
| ------------------------------------------------------ | ------- | -------- | ---- |
| <a href="#MAX_MAGS"></a> MAX_MAGS | `uint8` | 4        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MagWorkerData.msg)

:::details
Click here to see original file

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
