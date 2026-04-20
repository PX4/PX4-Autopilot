---
pageClass: is-wide-page
---

# EscStatus (UORB message)

**TOPICS:** esc_status

## Fields

| 参数名                                                        | 类型              | Unit [Frame] | Range/Enum                                                                            | 描述                                                                                 |
| ---------------------------------------------------------- | --------------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| timestamp                                                  | `uint64`        | us                                                               |                                                                                       | Time since system start                                                            |
| counter                                                    | `uint16`        |                                                                  |                                                                                       | Incremented by the writing thread everytime new data is stored                     |
| esc_count                             | `uint8`         |                                                                  |                                                                                       | Number of connected ESCs                                                           |
| esc_connectiontype                    | `uint8`         |                                                                  | [ESC_CONNECTION_TYPE](#ESC_CONNECTION_TYPE) | How ESCs connected to the system                                                   |
| esc_online_flags | `uint16`        |                                                                  |                                                                                       | Bitmask indicating which ESC is online/offline (in motor order) |
| esc_armed_flags  | `uint16`        |                                                                  |                                                                                       | Bitmask indicating which ESC is armed (in motor order)          |
| esc                                                        | `EscReport[12]` |                                                                  |                                                                                       |                                                                                    |

## Enums

### ESC_CONNECTION_TYPE {#ESC_CONNECTION_TYPE}

| 参数名                                                                                                                                  | 类型      | 值 | 描述                       |
| ------------------------------------------------------------------------------------------------------------------------------------ | ------- | - | ------------------------ |
| <a id="#ESC_CONNECTION_TYPE_PPM"></a> ESC_CONNECTION_TYPE_PPM         | `uint8` | 0 | Traditional PPM ESC      |
| <a id="#ESC_CONNECTION_TYPE_SERIAL"></a> ESC_CONNECTION_TYPE_SERIAL   | `uint8` | 1 | Serial Bus connected ESC |
| <a id="#ESC_CONNECTION_TYPE_ONESHOT"></a> ESC_CONNECTION_TYPE_ONESHOT | `uint8` | 2 | One Shot PPM             |
| <a id="#ESC_CONNECTION_TYPE_I2C"></a> ESC_CONNECTION_TYPE_I2C         | `uint8` | 3 | I2C                      |
| <a id="#ESC_CONNECTION_TYPE_CAN"></a> ESC_CONNECTION_TYPE_CAN         | `uint8` | 4 | CAN-Bus                  |
| <a id="#ESC_CONNECTION_TYPE_DSHOT"></a> ESC_CONNECTION_TYPE_DSHOT     | `uint8` | 5 | DShot                    |

## Constants

| 参数名                                                                                         | 类型      | 值  | 描述                                                               |
| ------------------------------------------------------------------------------------------- | ------- | -- | ---------------------------------------------------------------- |
| <a id="#CONNECTED_ESC_MAX"></a> CONNECTED_ESC_MAX | `uint8` | 12 | The number of ESCs supported (Motor1-Motor12) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EscStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp # [us] Time since system start
uint8 CONNECTED_ESC_MAX = 12 # The number of ESCs supported (Motor1-Motor12)

uint8 ESC_CONNECTION_TYPE_PPM = 0 # Traditional PPM ESC
uint8 ESC_CONNECTION_TYPE_SERIAL = 1 # Serial Bus connected ESC
uint8 ESC_CONNECTION_TYPE_ONESHOT = 2 # One Shot PPM
uint8 ESC_CONNECTION_TYPE_I2C = 3 # I2C
uint8 ESC_CONNECTION_TYPE_CAN = 4 # CAN-Bus
uint8 ESC_CONNECTION_TYPE_DSHOT = 5 # DShot

uint16 counter # [-] Incremented by the writing thread everytime new data is stored

uint8 esc_count # [-] Number of connected ESCs
uint8 esc_connectiontype # [@enum ESC_CONNECTION_TYPE] How ESCs connected to the system

uint16 esc_online_flags               # Bitmask indicating which ESC is online/offline (in motor order)
# esc_online_flags bit 0 : Set to 1 if Motor1 is online
# esc_online_flags bit 1 : Set to 1 if Motor2 is online
# esc_online_flags bit 2 : Set to 1 if Motor3 is online
# esc_online_flags bit 3 : Set to 1 if Motor4 is online
# esc_online_flags bit 4 : Set to 1 if Motor5 is online
# esc_online_flags bit 5 : Set to 1 if Motor6 is online
# esc_online_flags bit 6 : Set to 1 if Motor7 is online
# esc_online_flags bit 7 : Set to 1 if Motor8 is online
# esc_online_flags bit 8 : Set to 1 if Motor9 is online
# esc_online_flags bit 9 : Set to 1 if Motor10 is online
# esc_online_flags bit 10: Set to 1 if Motor11 is online
# esc_online_flags bit 11: Set to 1 if Motor12 is online

uint16 esc_armed_flags # [-] Bitmask indicating which ESC is armed (in motor order)

EscReport[12] esc
```

:::
