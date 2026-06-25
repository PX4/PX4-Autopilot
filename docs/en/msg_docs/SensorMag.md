---
pageClass: is-wide-page
---

# SensorMag (UORB message)

**TOPICS:** sensor_mag

## Fields

| Name                                              | Type      | Unit [Frame] | Range/Enum | Description                                                               |
| ------------------------------------------------- | --------- | ------------ | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`  |              |            | time since system start (microseconds)                                    |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`  |              |            |
| <a id="fld_device_id"></a>device_id               | `uint32`  |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_x"></a>x                               | `float32` |              |            | magnetic field in the FRD board frame X-axis in Gauss                     |
| <a id="fld_y"></a>y                               | `float32` |              |            | magnetic field in the FRD board frame Y-axis in Gauss                     |
| <a id="fld_z"></a>z                               | `float32` |              |            | magnetic field in the FRD board frame Z-axis in Gauss                     |
| <a id="fld_temperature"></a>temperature           | `float32` |              |            | temperature in degrees Celsius                                            |
| <a id="fld_error_count"></a>error_count           | `uint32`  |              |            |

## Constants

| Name                                            | Type    | Value | Description |
| ----------------------------------------------- | ------- | ----- | ----------- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorMag.msg)

::: details Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 x                 # magnetic field in the FRD board frame X-axis in Gauss
float32 y                 # magnetic field in the FRD board frame Y-axis in Gauss
float32 z                 # magnetic field in the FRD board frame Z-axis in Gauss

float32 temperature       # temperature in degrees Celsius

uint32 error_count

uint8 ORB_QUEUE_LENGTH = 4
```

:::
