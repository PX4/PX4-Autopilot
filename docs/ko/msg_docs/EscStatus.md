---
pageClass: is-wide-page
---

# EscStatus (UORB message)

**TOPICS:** esc_status

## Fields

| 명칭                                                         | 형식             | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                                                   |
| ---------------------------------------------------------- | -------------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                  | `uint64`       |                                                                  |            | time since system start (microseconds)                                                                                                                                            |
| counter                                                    | `uint16`       |                                                                  |            | incremented by the writing thread everytime new data is stored                                                                                                                                       |
| esc_count                             | `uint8`        |                                                                  |            | number of connected ESCs                                                                                                                                                                             |
| esc_connectiontype                    | `uint8`        |                                                                  |            | how ESCs connected to the system                                                                                                                                                                     |
| esc_online_flags | `uint8`        |                                                                  |            | Bitmask indicating which ESC is online/offline                                                                                                                                                       |
| esc_armed_flags  | `uint8`        |                                                                  |            | Bitmask indicating which ESC is armed. For ESC's where the arming state is not known (returned by the ESC), the arming bits should always be set. |
| esc                                                        | `EscReport[8]` |                                                                  |            |                                                                                                                                                                                                      |

## Constants

| 명칭                                                                                                                                     | 형식      | Value | 설명                                                                                                   |
| -------------------------------------------------------------------------------------------------------------------------------------- | ------- | ----- | ---------------------------------------------------------------------------------------------------- |
| <a href="#CONNECTED_ESC_MAX"></a> CONNECTED_ESC_MAX                                          | `uint8` | 8     | The number of ESCs supported. Current (Q2/2013) we support 8 ESCs |
| <a href="#ESC_CONNECTION_TYPE_PPM"></a> ESC_CONNECTION_TYPE_PPM         | `uint8` | 0     | Traditional PPM ESC                                                                                  |
| <a href="#ESC_CONNECTION_TYPE_SERIAL"></a> ESC_CONNECTION_TYPE_SERIAL   | `uint8` | 1     | Serial Bus connected ESC                                                                             |
| <a href="#ESC_CONNECTION_TYPE_ONESHOT"></a> ESC_CONNECTION_TYPE_ONESHOT | `uint8` | 2     | One Shot PPM                                                                                         |
| <a href="#ESC_CONNECTION_TYPE_I2C"></a> ESC_CONNECTION_TYPE_I2C         | `uint8` | 3     | I2C                                                                                                  |
| <a href="#ESC_CONNECTION_TYPE_CAN"></a> ESC_CONNECTION_TYPE_CAN         | `uint8` | 4     | CAN-Bus                                                                                              |
| <a href="#ESC_CONNECTION_TYPE_DSHOT"></a> ESC_CONNECTION_TYPE_DSHOT     | `uint8` | 5     | DShot                                                                                                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EscStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp					# time since system start (microseconds)
uint8 CONNECTED_ESC_MAX = 8				# The number of ESCs supported. Current (Q2/2013) we support 8 ESCs

uint8 ESC_CONNECTION_TYPE_PPM = 0			# Traditional PPM ESC
uint8 ESC_CONNECTION_TYPE_SERIAL = 1			# Serial Bus connected ESC
uint8 ESC_CONNECTION_TYPE_ONESHOT = 2			# One Shot PPM
uint8 ESC_CONNECTION_TYPE_I2C = 3			# I2C
uint8 ESC_CONNECTION_TYPE_CAN = 4			# CAN-Bus
uint8 ESC_CONNECTION_TYPE_DSHOT = 5			# DShot

uint16 counter  					# incremented by the writing thread everytime new data is stored

uint8 esc_count						# number of connected ESCs
uint8 esc_connectiontype				# how ESCs connected to the system

uint8 esc_online_flags					# Bitmask indicating which ESC is online/offline
# esc_online_flags bit 0 : Set to 1 if ESC0 is online
# esc_online_flags bit 1 : Set to 1 if ESC1 is online
# esc_online_flags bit 2 : Set to 1 if ESC2 is online
# esc_online_flags bit 3 : Set to 1 if ESC3 is online
# esc_online_flags bit 4 : Set to 1 if ESC4 is online
# esc_online_flags bit 5 : Set to 1 if ESC5 is online
# esc_online_flags bit 6 : Set to 1 if ESC6 is online
# esc_online_flags bit 7 : Set to 1 if ESC7 is online

uint8 esc_armed_flags					# Bitmask indicating which ESC is armed. For ESC's where the arming state is not known (returned by the ESC), the arming bits should always be set.

EscReport[8] esc
```

:::
