---
pageClass: is-wide-page
---

# VehicleImu (UORB message)

IMU readings in SI-unit form.

**TOPICS:** vehicle_imu

## Fields

| 参数名                                                               | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                    |
| ----------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                         | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                             |
| timestamp_sample                             | `uint64`     |                                                                  |            |                                                                                                                                                                       |
| accel_device_id         | `uint32`     |                                                                  |            | Accelerometer unique device ID for the sensor that does not change between power cycles                                                                               |
| gyro_device_id          | `uint32`     |                                                                  |            | Gyroscope unique device ID for the sensor that does not change between power cycles                                                                                   |
| delta_angle                                  | `float32[3]` |                                                                  |            | delta angle about the FRD body frame XYZ-axis in rad over the integration time frame (delta_angle_dt)    |
| delta_velocity                               | `float32[3]` |                                                                  |            | delta velocity in the FRD body frame XYZ-axis in m/s over the integration time frame (delta_velocity_dt) |
| delta_angle_dt          | `uint32`     |                                                                  |            | integration period in microseconds                                                                                                                                    |
| delta_velocity_dt       | `uint32`     |                                                                  |            | integration period in microseconds                                                                                                                                    |
| delta_angle_clipping    | `uint8`      |                                                                  |            | bitfield indicating if there was any gyro clipping (per axis) during the integration time frame                                                    |
| delta_velocity_clipping | `uint8`      |                                                                  |            | bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame                                           |
| accel_calibration_count | `uint8`      |                                                                  |            | Calibration changed counter. Monotonically increases whenever accelermeter calibration changes.                                       |
| gyro_calibration_count  | `uint8`      |                                                                  |            | Calibration changed counter. Monotonically increases whenever rate gyro calibration changes.                                          |

## Constants

| 参数名                                                        | 类型      | 值 | 描述 |
| ---------------------------------------------------------- | ------- | - | -- |
| <a href="#CLIPPING_X"></a> CLIPPING_X | `uint8` | 1 |    |
| <a href="#CLIPPING_Y"></a> CLIPPING_Y | `uint8` | 2 |    |
| <a href="#CLIPPING_Z"></a> CLIPPING_Z | `uint8` | 4 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleImu.msg)

:::details
Click here to see original file

```c
# IMU readings in SI-unit form.

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 accel_device_id          # Accelerometer unique device ID for the sensor that does not change between power cycles
uint32 gyro_device_id           # Gyroscope unique device ID for the sensor that does not change between power cycles

float32[3] delta_angle          # delta angle about the FRD body frame XYZ-axis in rad over the integration time frame (delta_angle_dt)
float32[3] delta_velocity       # delta velocity in the FRD body frame XYZ-axis in m/s over the integration time frame (delta_velocity_dt)

uint32 delta_angle_dt           # integration period in microseconds
uint32 delta_velocity_dt        # integration period in microseconds

uint8 CLIPPING_X = 1
uint8 CLIPPING_Y = 2
uint8 CLIPPING_Z = 4

uint8 delta_angle_clipping     # bitfield indicating if there was any gyro clipping (per axis) during the integration time frame
uint8 delta_velocity_clipping   # bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame

uint8 accel_calibration_count  	# Calibration changed counter. Monotonically increases whenever accelermeter calibration changes.
uint8 gyro_calibration_count   	# Calibration changed counter. Monotonically increases whenever rate gyro calibration changes.
```

:::
