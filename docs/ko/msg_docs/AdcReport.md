---
pageClass: is-wide-page
---

# AdcReport (UORB message)

ADC raw data.

Communicates raw data from an analog-to-digital converter (ADC) to other modules, such as battery status.

**TOPICS:** adc_report

## Fields

| 명칭                              | 형식          | Unit [Frame] | Range/Enum | 설명                                                                                                   |
| ------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------- |
| timestamp                       | `uint64`    | us                                                               |            | Time since system start                                                                              |
| device_id  | `uint32`    |                                                                  |            | unique device ID for the sensor that does not change between power cycles                            |
| channel_id | `int16[16]` |                                                                  |            | ADC channel IDs, negative for non-existent, TODO: should be kept same as array index |
| raw_data   | `int32[16]` |                                                                  |            | ADC channel raw value, accept negative value, valid if channel ID is positive                        |
| resolution                      | `uint32`    |                                                                  |            | ADC channel resolution                                                                               |
| v_ref      | `float32`   | V                                                                |            | ADC channel voltage reference, use to calculate LSB voltage(lsb=scale/resolution) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/AdcReport.msg)

:::details
Click here to see original file

```c
# ADC raw data.
#
# Communicates raw data from an analog-to-digital converter (ADC) to other modules, such as battery status.

uint64 timestamp      # [us] Time since system start
uint32 device_id      # [-] unique device ID for the sensor that does not change between power cycles
int16[16] channel_id  # [-] ADC channel IDs, negative for non-existent, TODO: should be kept same as array index
int32[16] raw_data    # [-] ADC channel raw value, accept negative value, valid if channel ID is positive
uint32 resolution     # [-] ADC channel resolution
float32 v_ref         # [V] ADC channel voltage reference, use to calculate LSB voltage(lsb=scale/resolution)
```

:::
