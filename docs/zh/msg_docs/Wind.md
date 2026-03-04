---
pageClass: is-wide-page
---

# Wind (UORB message)

Wind estimate (from EKF2).

Contains the system-wide estimate of horizontal wind velocity and its variance.
Published by the navigation filter (EKF2) for use by other flight modules and libraries.

**TOPICS:** wind estimator_wind

## Fields

| 参数名                                                      | 类型        | Unit [Frame] | Range/Enum | 描述                                                                                                                   |
| -------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                | `uint64`  | us                                                               |            | Time since system start                                                                                              |
| timestamp_sample                    | `uint64`  | us                                                               |            | Timestamp of the raw data                                                                                            |
| windspeed_north                     | `float32` | 米/秒                                                              |            | Wind component in north / X direction                                                                                |
| windspeed_east                      | `float32` | 米/秒                                                              |            | Wind component in east / Y direction                                                                                 |
| variance_north                      | `float32` | (m/s)^2                                       |            | Wind estimate error variance in north / X direction (Invalid: 0 if not estimated) |
| variance_east                       | `float32` | (m/s)^2                                       |            | Wind estimate error variance in east / Y direction (Invalid: 0 if not estimated)  |
| tas_innov                           | `float32` | 米/秒                                                              |            | True airspeed innovation                                                                                             |
| tas_innov_var  | `float32` | (m/s)^2                                       |            | True airspeed innovation variance                                                                                    |
| beta_innov                          | `float32` | rad                                                              |            | Sideslip measurement innovation                                                                                      |
| beta_innov_var | `float32` | rad^2                                                            |            | Sideslip measurement innovation variance                                                                             |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述 |
| -------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/Wind.msg)

:::details
Click here to see original file

```c
# Wind estimate (from EKF2)
#
# Contains the system-wide estimate of horizontal wind velocity and its variance.
# Published by the navigation filter (EKF2) for use by other flight modules and libraries.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

float32 windspeed_north # [m/s] Wind component in north / X direction
float32 windspeed_east # [m/s] Wind component in east / Y direction

float32 variance_north # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in north / X direction
float32 variance_east # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in east / Y direction

float32 tas_innov # [m/s] True airspeed innovation
float32 tas_innov_var # [(m/s)^2] True airspeed innovation variance

float32 beta_innov # [rad] Sideslip measurement innovation
float32 beta_innov_var # [rad^2] Sideslip measurement innovation variance

# TOPICS wind estimator_wind
```

:::
