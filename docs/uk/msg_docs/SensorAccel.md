---
pageClass: is-wide-page
---

# SensorAccel (UORB message)

**TOPICS:** sensor_accel

## Fields

| Назва                                 | Тип        | Unit [Frame] | Range/Enum | Опис                                                                      |
| ------------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| timestamp                             | `uint64`   |                                                                  |            | time since system start (microseconds)                 |
| timestamp_sample | `uint64`   |                                                                  |            |                                                                           |
| device_id        | `uint32`   |                                                                  |            | unique device ID for the sensor that does not change between power cycles |
| x                                     | `float32`  |                                                                  |            | acceleration in the FRD board frame X-axis in m/s^2                       |
| y                                     | `float32`  |                                                                  |            | acceleration in the FRD board frame Y-axis in m/s^2                       |
| z                                     | `float32`  |                                                                  |            | acceleration in the FRD board frame Z-axis in m/s^2                       |
| temperature                           | `float32`  |                                                                  |            | temperature in degrees Celsius                                            |
| error_count      | `uint32`   |                                                                  |            |                                                                           |
| clip_counter     | `uint8[3]` |                                                                  |            | clip count per axis in the sample period                                  |
| samples                               | `uint8`    |                                                                  |            | number of raw samples that went into this message                         |

## Constants

| Назва                                                                                       | Тип     | Значення | Опис |
| ------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 8        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorAccel.msg)

:::details
Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 x                 # acceleration in the FRD board frame X-axis in m/s^2
float32 y                 # acceleration in the FRD board frame Y-axis in m/s^2
float32 z                 # acceleration in the FRD board frame Z-axis in m/s^2

float32 temperature       # temperature in degrees Celsius

uint32 error_count

uint8[3] clip_counter     # clip count per axis in the sample period

uint8 samples             # number of raw samples that went into this message

uint8 ORB_QUEUE_LENGTH = 8
```

:::
