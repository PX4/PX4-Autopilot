---
pageClass: is-wide-page
---

# EscEepromWrite (UORB message)

**TOPICS:** esc_eeprom_write

## Fields

| 参数名                             | 类型          | Unit [Frame] | Range/Enum | 描述                                                                                                    |
| ------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------- |
| timestamp                       | `uint64`    | us                                                               |            | Time since system start                                                                               |
| firmware                        | `uint8`     |                                                                  |            | ESC firmware type (see ESC_FIRMWARE enum in MAVLink)          |
| index                           | `uint8`     |                                                                  |            | Index of the ESC (0 = ESC1, 1 = ESC2, etc, 255 = All)                              |
| length                          | `uint16`    |                                                                  |            | Length of valid data                                                                                  |
| data                            | `uint8[48]` |                                                                  |            | Raw ESC EEPROM data                                                                                   |
| write_mask | `uint32[2]` |                                                                  |            | Bitmask indicating which bytes in the data array should be written (max 48 values) |

## Constants

| 参数名                                                                                       | 类型      | 值 | 描述                              |
| ----------------------------------------------------------------------------------------- | ------- | - | ------------------------------- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 8 | To support 8 queued up requests |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EscEepromWrite.msg)

:::details
Click here to see original file

```c
uint64 timestamp # [us] Time since system start
uint8 firmware # [-] ESC firmware type (see ESC_FIRMWARE enum in MAVLink)
uint8 index # [-] Index of the ESC (0 = ESC1, 1 = ESC2, etc, 255 = All)
uint16 length # [-] Length of valid data
uint8[48] data # [-] Raw ESC EEPROM data
uint32[2] write_mask # [-] Bitmask indicating which bytes in the data array should be written (max 48 values)

uint8 ORB_QUEUE_LENGTH = 8 # To support 8 queued up requests
```

:::
