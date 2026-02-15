---
pageClass: is-wide-page
---

# SensorCombined (UORB message)

Sensor readings in SI-unit form. These fields are scaled and offset-compensated where possible and do not. change with board revisions and sensor updates.

**TOPICS:** sensor_combined

## Fields

| 参数名                                                                        | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                                                              |
| -------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                  | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                       |
| gyro_rad                                              | `float32[3]` |                                                                  |            | average angular rate measured in the FRD body frame XYZ-axis in rad/s over the last gyro sampling period                        |
| gyro_integral_dt                 | `uint32`     |                                                                  |            | gyro measurement sampling period in microseconds                                                                                |
| accelerometer_timestamp_relative | `int32`      |                                                                  |            | timestamp + accelerometer_timestamp_relative = Accelerometer timestamp                |
| accelerometer_m_s2               | `float32[3]` |                                                                  |            | average value acceleration measured in the FRD body frame XYZ-axis in m/s^2 over the last accelerometer sampling period         |
| accelerometer_integral_dt        | `uint32`     |                                                                  |            | accelerometer measurement sampling period in microseconds                                                                       |
| accelerometer_clipping                                | `uint8`      |                                                                  |            | bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame     |
| gyro_clipping                                         | `uint8`      |                                                                  |            | bitfield indicating if there was any gyro clipping (per axis) during the integration time frame              |
| accel_calibration_count          | `uint8`      |                                                                  |            | Calibration changed counter. Monotonically increases whenever accelermeter calibration changes. |
| gyro_calibration_count           | `uint8`      |                                                                  |            | Calibration changed counter. Monotonically increases whenever rate gyro calibration changes.    |

## Constants

| 参数名                                                                                                             | 类型      | 值          | 描述                                                                                                                                        |
| --------------------------------------------------------------------------------------------------------------- | ------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#RELATIVE_TIMESTAMP_INVALID"></a> RELATIVE_TIMESTAMP_INVALID | `int32` | 2147483647 | (0x7fffffff) If one of the relative timestamps is set to this value, it means the associated sensor values are invalid |
| <a href="#CLIPPING_X"></a> CLIPPING_X                                                      | `uint8` | 1          |                                                                                                                                           |
| <a href="#CLIPPING_Y"></a> CLIPPING_Y                                                      | `uint8` | 2          |                                                                                                                                           |
| <a href="#CLIPPING_Z"></a> CLIPPING_Z                                                      | `uint8` | 4          |                                                                                                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorCombined.msg)

:::details
Click here to see original file

```c
# Sensor readings in SI-unit form.
# These fields are scaled and offset-compensated where possible and do not
# change with board revisions and sensor updates.

uint64 timestamp                                # time since system start (microseconds)

int32 RELATIVE_TIMESTAMP_INVALID = 2147483647   # (0x7fffffff) If one of the relative timestamps is set to this value, it means the associated sensor values are invalid

# gyro timstamp is equal to the timestamp of the message
float32[3] gyro_rad                     # average angular rate measured in the FRD body frame XYZ-axis in rad/s over the last gyro sampling period
uint32 gyro_integral_dt                 # gyro measurement sampling period in microseconds

int32 accelerometer_timestamp_relative  # timestamp + accelerometer_timestamp_relative = Accelerometer timestamp
float32[3] accelerometer_m_s2           # average value acceleration measured in the FRD body frame XYZ-axis in m/s^2 over the last accelerometer sampling period
uint32 accelerometer_integral_dt        # accelerometer measurement sampling period in microseconds

uint8 CLIPPING_X = 1
uint8 CLIPPING_Y = 2
uint8 CLIPPING_Z = 4

uint8 accelerometer_clipping    # bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame
uint8 gyro_clipping             # bitfield indicating if there was any gyro clipping (per axis) during the integration time frame

uint8 accel_calibration_count   # Calibration changed counter. Monotonically increases whenever accelermeter calibration changes.
uint8 gyro_calibration_count    # Calibration changed counter. Monotonically increases whenever rate gyro calibration changes.
```

:::
