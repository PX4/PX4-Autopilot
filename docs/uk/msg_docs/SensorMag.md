---
pageClass: is-wide-page
---

# SensorMag (повідомлення UORB)

**TOPICS:** sensor_mag

## Fields

| Назва                                 | Тип       | Unit [Frame] | Range/Enum | Опис                                                                      |
| ------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| timestamp                             | `uint64`  |                                                                  |            | time since system start (microseconds)                 |
| timestamp_sample | `uint64`  |                                                                  |            |                                                                           |
| device_id        | `uint32`  |                                                                  |            | unique device ID for the sensor that does not change between power cycles |
| x                                     | `float32` |                                                                  |            | magnetic field in the FRD board frame X-axis in Gauss                     |
| y                                     | `float32` |                                                                  |            | magnetic field in the FRD board frame Y-axis in Gauss                     |
| z                                     | `float32` |                                                                  |            | magnetic field in the FRD board frame Z-axis in Gauss                     |
| temperature                           | `float32` |                                                                  |            | temperature in degrees Celsius                                            |
| error_count      | `uint32`  |                                                                  |            |                                                                           |

## Constants

| Назва                                                                                       | Тип     | Значення | Опис |
| ------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorMag.msg)

:::details
Click here to see original file

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
