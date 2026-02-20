---
pageClass: is-wide-page
---

# SensorGyroFft (UORB message)

**TOPICS:** sensor_gyrofft

## Fields

| 参数名                                                                                  | 类型           | Unit [Frame] | Range/Enum | 描述                                                                        |
| ------------------------------------------------------------------------------------ | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| timestamp                                                                            | `uint64`     |                                                                  |            | time since system start (microseconds)                 |
| timestamp_sample                                                | `uint64`     |                                                                  |            |                                                                           |
| device_id                                                       | `uint32`     |                                                                  |            | unique device ID for the sensor that does not change between power cycles |
| sensor_sample_rate_hz | `float32`    |                                                                  |            |                                                                           |
| resolution_hz                                                   | `float32`    |                                                                  |            |                                                                           |
| peak_frequencies_x                         | `float32[3]` |                                                                  |            | x axis peak frequencies                                                   |
| peak_frequencies_y                         | `float32[3]` |                                                                  |            | y axis peak frequencies                                                   |
| peak_frequencies_z                         | `float32[3]` |                                                                  |            | z axis peak frequencies                                                   |
| peak_snr_x                                 | `float32[3]` |                                                                  |            | x axis peak SNR                                                           |
| peak_snr_y                                 | `float32[3]` |                                                                  |            | y axis peak SNR                                                           |
| peak_snr_z                                 | `float32[3]` |                                                                  |            | z axis peak SNR                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGyroFft.msg)

:::details
Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 sensor_sample_rate_hz
float32 resolution_hz

float32[3] peak_frequencies_x # x axis peak frequencies
float32[3] peak_frequencies_y # y axis peak frequencies
float32[3] peak_frequencies_z # z axis peak frequencies

float32[3] peak_snr_x # x axis peak SNR
float32[3] peak_snr_y # y axis peak SNR
float32[3] peak_snr_z # z axis peak SNR
```

:::
