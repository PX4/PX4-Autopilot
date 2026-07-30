---
pageClass: is-wide-page
---

# SensorMag (повідомлення UORB)

**TOPICS:** sensor_mag

## Fields

| Назва                                                                  | Тип       | Unit [Frame] | Range/Enum | Опис                                                                      |
| ---------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                    | `uint64`  |                                                                  |            | time since system start (microseconds)                 |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`  |                                                                  |            |                                                                           |
| <a id="fld_device_id"></a>device_id               | `uint32`  |                                                                  |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_x"></a>x                                                    | `float32` | Gauss                                                            |            | magnetic field in the FRD board frame X-axis                              |
| <a id="fld_y"></a>y                                                    | `float32` | Gauss                                                            |            | magnetic field in the FRD board frame Y-axis                              |
| <a id="fld_z"></a>z                                                    | `float32` | Gauss                                                            |            | magnetic field in the FRD board frame Z-axis                              |
| <a id="fld_temperature"></a>temperature                                | `float32` | °C                                                               |            | Temperature.                                              |
| <a id="fld_error_count"></a>error_count           | `uint32`  |                                                                  |            |                                                                           |

## Constants

| Назва                                                                                     | Тип     | Значення | Опис |
| ----------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorMag.msg)

:::details
Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 x                 # [Gauss] magnetic field in the FRD board frame X-axis
float32 y                 # [Gauss] magnetic field in the FRD board frame Y-axis
float32 z                 # [Gauss] magnetic field in the FRD board frame Z-axis

float32 temperature       # [°C] Temperature.

uint32 error_count

uint8 ORB_QUEUE_LENGTH = 4
```

:::
