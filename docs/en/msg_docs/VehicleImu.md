---
pageClass: is-wide-page
---

# VehicleImu (UORB message)

IMU readings in SI-unit form.

**TOPICS:** vehicle_imu

## Fields

| Name                                                            | Type         | Unit [Frame] | Range/Enum | Description                                                                                              |
| --------------------------------------------------------------- | ------------ | ------------ | ---------- | -------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                             | `uint64`     |              |            | time since system start (microseconds)                                                                   |
| <a id="fld_timestamp_sample"></a>timestamp_sample               | `uint64`     |              |            |
| <a id="fld_accel_device_id"></a>accel_device_id                 | `uint32`     |              |            | Accelerometer unique device ID for the sensor that does not change between power cycles                  |
| <a id="fld_gyro_device_id"></a>gyro_device_id                   | `uint32`     |              |            | Gyroscope unique device ID for the sensor that does not change between power cycles                      |
| <a id="fld_delta_angle"></a>delta_angle                         | `float32[3]` |              |            | delta angle about the FRD body frame XYZ-axis in rad over the integration time frame (delta_angle_dt)    |
| <a id="fld_delta_velocity"></a>delta_velocity                   | `float32[3]` |              |            | delta velocity in the FRD body frame XYZ-axis in m/s over the integration time frame (delta_velocity_dt) |
| <a id="fld_delta_angle_dt"></a>delta_angle_dt                   | `uint32`     |              |            | integration period in microseconds                                                                       |
| <a id="fld_delta_velocity_dt"></a>delta_velocity_dt             | `uint32`     |              |            | integration period in microseconds                                                                       |
| <a id="fld_delta_angle_clipping"></a>delta_angle_clipping       | `uint8`      |              |            | bitfield indicating if there was any gyro clipping (per axis) during the integration time frame          |
| <a id="fld_delta_velocity_clipping"></a>delta_velocity_clipping | `uint8`      |              |            | bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame |
| <a id="fld_accel_calibration_count"></a>accel_calibration_count | `uint8`      |              |            | Calibration changed counter. Monotonically increases whenever accelermeter calibration changes.          |
| <a id="fld_gyro_calibration_count"></a>gyro_calibration_count   | `uint8`      |              |            | Calibration changed counter. Monotonically increases whenever rate gyro calibration changes.             |

## Constants

| Name                                | Type    | Value | Description |
| ----------------------------------- | ------- | ----- | ----------- |
| <a id="#CLIPPING_X"></a> CLIPPING_X | `uint8` | 1     |
| <a id="#CLIPPING_Y"></a> CLIPPING_Y | `uint8` | 2     |
| <a id="#CLIPPING_Z"></a> CLIPPING_Z | `uint8` | 4     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleImu.msg)

::: details Click here to see original file

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
