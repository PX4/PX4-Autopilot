---
pageClass: is-wide-page
---

# MavlinkTunnel (UORB message)

MAV_TUNNEL_PAYLOAD_TYPE enum.

**TOPICS:** mavlink_tunnel esc_serial_passthru io_serial_passthru

## Fields

| 参数名                                   | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`     |                                                                  |            | Time since system start (microseconds)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| payload_type     | `uint16`     |                                                                  |            | A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase. |
| target_system    | `uint8`      |                                                                  |            | System ID (can be 0 for broadcast, but this is discouraged)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| target_component | `uint8`      |                                                                  |            | Component ID (can be 0 for broadcast, but this is discouraged)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| payload_length   | `uint8`      |                                                                  |            | Length of the data transported in payload                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| payload                               | `uint8[128]` |                                                                  |            | Data itself                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

## Constants

| 参数名                                                                                                                                                                                                          | 类型      | 值   | 描述                                       |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------- | --- | ---------------------------------------- |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN"></a> MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN                                          | `uint8` | 0   | Encoding of payload unknown              |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 | `uint8` | 200 | Registered for STorM32 gimbal controller |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 | `uint8` | 201 | Registered for STorM32 gimbal controller |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 | `uint8` | 202 | Registered for STorM32 gimbal controller |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 | `uint8` | 203 | Registered for STorM32 gimbal controller |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 | `uint8` | 204 | Registered for STorM32 gimbal controller |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 | `uint8` | 205 | Registered for STorM32 gimbal controller |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 | `uint8` | 206 | Registered for STorM32 gimbal controller |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 | `uint8` | 207 | Registered for STorM32 gimbal controller |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 | `uint8` | 208 | Registered for STorM32 gimbal controller |
| <a href="#MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9"></a> MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 | `uint8` | 209 | Registered for STorM32 gimbal controller |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MavlinkTunnel.msg)

:::details
Click here to see original file

```c
# MAV_TUNNEL_PAYLOAD_TYPE enum

uint8 MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN = 0                # Encoding of payload unknown
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 = 200    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 = 201    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 = 202    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 = 203    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 = 204    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 = 205    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 = 206    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 = 207    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 = 208    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 = 209    # Registered for STorM32 gimbal controller

uint64 timestamp	     # Time since system start (microseconds)
uint16 payload_type      # A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
uint8 target_system      # System ID (can be 0 for broadcast, but this is discouraged)
uint8 target_component   # Component ID (can be 0 for broadcast, but this is discouraged)
uint8 payload_length     # Length of the data transported in payload
uint8[128] payload       # Data itself

# Topic aliases for known payload types
# TOPICS mavlink_tunnel esc_serial_passthru io_serial_passthru
```

:::
