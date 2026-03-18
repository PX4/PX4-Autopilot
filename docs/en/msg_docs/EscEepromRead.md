---
pageClass: is-wide-page
---

# EscEepromRead (UORB message)

**TOPICS:** esc_eeprom_read

## Fields

| Name      | Type        | Unit [Frame] | Range/Enum | Description                                          |
| --------- | ----------- | ------------ | ---------- | ---------------------------------------------------- |
| timestamp | `uint64`    | us           |            | Time since system start                              |
| firmware  | `uint8`     |              |            | ESC firmware type (see ESC_FIRMWARE enum in MAVLink) |
| index     | `uint8`     |              |            | Index of the ESC (0 = ESC1, 1 = ESC2, etc.)          |
| length    | `uint16`    |              |            | Length of valid data                                 |
| data      | `uint8[48]` |              |            | Raw ESC EEPROM data                                  |

## Constants

| Name                                            | Type    | Value | Description                      |
| ----------------------------------------------- | ------- | ----- | -------------------------------- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 8     | To support 8 queued up responses |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EscEepromRead.msg)

::: details Click here to see original file

```c
uint64 timestamp # [us] Time since system start
uint8 firmware # [-] ESC firmware type (see ESC_FIRMWARE enum in MAVLink)
uint8 index # [-] Index of the ESC (0 = ESC1, 1 = ESC2, etc.)
uint16 length # [-] Length of valid data
uint8[48] data # [-] Raw ESC EEPROM data

uint8 ORB_QUEUE_LENGTH = 8 # To support 8 queued up responses
```

:::
