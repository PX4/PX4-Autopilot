---
pageClass: is-wide-page
---

# SensorGyro (UORB message)

**TOPICS:** sensor_gyro

## Fields

| 명칭                                    | 형식         | Unit [Frame] | Range/Enum | 설명                                                                        |
| ------------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| timestamp                             | `uint64`   |                                                                  |            | time since system start (microseconds)                 |
| timestamp_sample | `uint64`   |                                                                  |            |                                                                           |
| device_id        | `uint32`   |                                                                  |            | unique device ID for the sensor that does not change between power cycles |
| x                                     | `float32`  |                                                                  |            | angular velocity in the FRD board frame X-axis in rad/s                   |
| y                                     | `float32`  |                                                                  |            | angular velocity in the FRD board frame Y-axis in rad/s                   |
| z                                     | `float32`  |                                                                  |            | angular velocity in the FRD board frame Z-axis in rad/s                   |
| temperature                           | `float32`  |                                                                  |            | temperature in degrees Celsius                                            |
| error_count      | `uint32`   |                                                                  |            |                                                                           |
| clip_counter     | `uint8[3]` |                                                                  |            | clip count per axis in the sample period                                  |
| samples                               | `uint8`    |                                                                  |            | number of raw samples that went into this message                         |

## Constants

| 명칭                                                                                          | 형식      | Value | 설명 |
| ------------------------------------------------------------------------------------------- | ------- | ----- | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 8     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGyro.msg)

:::details
Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 x                 # angular velocity in the FRD board frame X-axis in rad/s
float32 y                 # angular velocity in the FRD board frame Y-axis in rad/s
float32 z                 # angular velocity in the FRD board frame Z-axis in rad/s

float32 temperature       # temperature in degrees Celsius

uint32 error_count

uint8[3] clip_counter     # clip count per axis in the sample period

uint8 samples             # number of raw samples that went into this message

uint8 ORB_QUEUE_LENGTH = 8
```

:::
