---
pageClass: is-wide-page
---

# Vtx (UORB message)

**TOPICS:** vtx

## Fields

| 参数名                              | 类型          | Unit [Frame] | Range/Enum | 描述                                                                                |
| -------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------- |
| timestamp                        | `uint64`    |                                                                  |            | time since system start (microseconds)                         |
| protocol                         | `uint8`     |                                                                  |            |                                                                                   |
| device                           | `uint8`     |                                                                  |            |                                                                                   |
| mode                             | `uint8`     |                                                                  |            |                                                                                   |
| band                             | `int8`      |                                                                  |            | Band number (0-23), negative values indicate frequency mode    |
| channel                          | `int8`      |                                                                  |            | Channel number (0-15), negative values indicate frequency mode |
| frequency                        | `uint16`    |                                                                  |            | Frequency in MHz, zero indicates unknown                                          |
| band_letter | `uint8`     |                                                                  |            | Band letter as ASCII                                                              |
| band_name   | `uint8[12]` |                                                                  |            | Band name in ASCII without null termination                                       |
| power_level | `int8`      |                                                                  |            | Current power level (0-15), negative values indicate unknown   |
| power_label | `uint8[4]`  |                                                                  |            | Current power label in ASCII without null termination                             |

## Constants

| 参数名                                                                                                                                                     | 类型      | 值   | 描述                                        |
| ------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | --- | ----------------------------------------- |
| <a href="#BAND_NAME_LENGTH"></a> BAND_NAME_LENGTH                                                             | `uint8` | 12  |                                           |
| <a href="#POWER_LABEL_LENGTH"></a> POWER_LABEL_LENGTH                                                         | `uint8` | 4   |                                           |
| <a href="#PROTOCOL_NONE"></a> PROTOCOL_NONE                                                                                        | `uint8` | 0   | No protocol is detected, usually an error |
| <a href="#PROTOCOL_SMART_AUDIO_V1"></a> PROTOCOL_SMART_AUDIO_V1                          | `uint8` | 10  |                                           |
| <a href="#PROTOCOL_SMART_AUDIO_V2"></a> PROTOCOL_SMART_AUDIO_V2                          | `uint8` | 20  |                                           |
| <a href="#PROTOCOL_SMART_AUDIO_V2_1"></a> PROTOCOL_SMART_AUDIO_V2_1 | `uint8` | 21  |                                           |
| <a href="#PROTOCOL_TRAMP"></a> PROTOCOL_TRAMP                                                                                      | `uint8` | 100 |                                           |
| <a href="#DEVICE_UNKNOWN"></a> DEVICE_UNKNOWN                                                                                      | `uint8` | 0   |                                           |
| <a href="#DEVICE_PEAK_THOR_T67"></a> DEVICE_PEAK_THOR_T67                                | `uint8` | 20  |                                           |
| <a href="#DEVICE_RUSH_MAX_SOLO"></a> DEVICE_RUSH_MAX_SOLO                                | `uint8` | 40  |                                           |
| <a href="#MODE_NORMAL"></a> MODE_NORMAL                                                                                            | `uint8` | 0   |                                           |
| <a href="#MODE_PIT"></a> MODE_PIT                                                                                                  | `uint8` | 1   |                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Vtx.msg)

:::details
Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)

uint8 BAND_NAME_LENGTH = 12
uint8 POWER_LABEL_LENGTH = 4

uint8 PROTOCOL_NONE = 0              # No protocol is detected, usually an error
uint8 PROTOCOL_SMART_AUDIO_V1 = 10
uint8 PROTOCOL_SMART_AUDIO_V2 = 20
uint8 PROTOCOL_SMART_AUDIO_V2_1 = 21
uint8 PROTOCOL_TRAMP = 100
uint8 protocol

uint8 DEVICE_UNKNOWN = 0
uint8 DEVICE_PEAK_THOR_T67 = 20
uint8 DEVICE_RUSH_MAX_SOLO = 40
uint8 device

uint8 MODE_NORMAL = 0
uint8 MODE_PIT = 1
uint8 mode

# Band and Channel are 0-indexed! But the user expects a 1-indexed display!
int8 band        # Band number (0-23), negative values indicate frequency mode
int8 channel     # Channel number (0-15), negative values indicate frequency mode
uint16 frequency # Frequency in MHz, zero indicates unknown

uint8 band_letter   # Band letter as ASCII
uint8[12] band_name # Band name in ASCII without null termination

# Also 0-indexed, but the user expects a 1-indexed display!
int8 power_level     # Current power level (0-15), negative values indicate unknown
uint8[4] power_label # Current power label in ASCII without null termination
```

:::
