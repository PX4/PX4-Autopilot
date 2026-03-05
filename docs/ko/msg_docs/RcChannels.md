---
pageClass: is-wide-page
---

# RcChannels (UORB message)

**TOPICS:** rc_channels

## Fields

| 명칭                                                             | 형식            | Unit [Frame] | Range/Enum | 설명                                                                                                                                  |
| -------------------------------------------------------------- | ------------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                      | `uint64`      |                                                                  |            | time since system start (microseconds)                                                                           |
| timestamp_last_valid | `uint64`      |                                                                  |            | Timestamp of last valid RC signal                                                                                                   |
| channels                                                       | `float32[18]` |                                                                  |            | Scaled to -1..1 (throttle: 0..1) |
| channel_count                             | `uint8`       |                                                                  |            | Number of valid channels                                                                                                            |
| function                                                       | `int8[30]`    |                                                                  |            | Functions mapping                                                                                                                   |
| rssi                                                           | `uint8`       |                                                                  |            | Receive signal strength index                                                                                                       |
| signal_lost                               | `bool`        |                                                                  |            | Control signal lost, should be checked together with topic timeout                                                                  |
| frame_drop_count     | `uint32`      |                                                                  |            | Number of dropped frames                                                                                                            |

## Constants

| 명칭                                                                                                                                   | 형식      | Value | 설명 |
| ------------------------------------------------------------------------------------------------------------------------------------ | ------- | ----- | -- |
| <a href="#FUNCTION_THROTTLE"></a> FUNCTION_THROTTLE                                                             | `uint8` | 0     |    |
| <a href="#FUNCTION_ROLL"></a> FUNCTION_ROLL                                                                     | `uint8` | 1     |    |
| <a href="#FUNCTION_PITCH"></a> FUNCTION_PITCH                                                                   | `uint8` | 2     |    |
| <a href="#FUNCTION_YAW"></a> FUNCTION_YAW                                                                       | `uint8` | 3     |    |
| <a href="#FUNCTION_RETURN"></a> FUNCTION_RETURN                                                                 | `uint8` | 4     |    |
| <a href="#FUNCTION_LOITER"></a> FUNCTION_LOITER                                                                 | `uint8` | 5     |    |
| <a href="#FUNCTION_OFFBOARD"></a> FUNCTION_OFFBOARD                                                             | `uint8` | 6     |    |
| <a href="#FUNCTION_FLAPS"></a> FUNCTION_FLAPS                                                                   | `uint8` | 7     |    |
| <a href="#FUNCTION_AUX_1"></a> FUNCTION_AUX_1                                              | `uint8` | 8     |    |
| <a href="#FUNCTION_AUX_2"></a> FUNCTION_AUX_2                                              | `uint8` | 9     |    |
| <a href="#FUNCTION_AUX_3"></a> FUNCTION_AUX_3                                              | `uint8` | 10    |    |
| <a href="#FUNCTION_AUX_4"></a> FUNCTION_AUX_4                                              | `uint8` | 11    |    |
| <a href="#FUNCTION_AUX_5"></a> FUNCTION_AUX_5                                              | `uint8` | 12    |    |
| <a href="#FUNCTION_AUX_6"></a> FUNCTION_AUX_6                                              | `uint8` | 13    |    |
| <a href="#FUNCTION_PARAM_1"></a> FUNCTION_PARAM_1                                          | `uint8` | 14    |    |
| <a href="#FUNCTION_PARAM_2"></a> FUNCTION_PARAM_2                                          | `uint8` | 15    |    |
| <a href="#FUNCTION_PARAM_3_5"></a> FUNCTION_PARAM_3_5                 | `uint8` | 16    |    |
| <a href="#FUNCTION_KILLSWITCH"></a> FUNCTION_KILLSWITCH                                                         | `uint8` | 17    |    |
| <a href="#FUNCTION_TRANSITION"></a> FUNCTION_TRANSITION                                                         | `uint8` | 18    |    |
| <a href="#FUNCTION_GEAR"></a> FUNCTION_GEAR                                                                     | `uint8` | 19    |    |
| <a href="#FUNCTION_ARMSWITCH"></a> FUNCTION_ARMSWITCH                                                           | `uint8` | 20    |    |
| <a href="#FUNCTION_FLTBTN_SLOT_1"></a> FUNCTION_FLTBTN_SLOT_1         | `uint8` | 21    |    |
| <a href="#FUNCTION_FLTBTN_SLOT_2"></a> FUNCTION_FLTBTN_SLOT_2         | `uint8` | 22    |    |
| <a href="#FUNCTION_FLTBTN_SLOT_3"></a> FUNCTION_FLTBTN_SLOT_3         | `uint8` | 23    |    |
| <a href="#FUNCTION_FLTBTN_SLOT_4"></a> FUNCTION_FLTBTN_SLOT_4         | `uint8` | 24    |    |
| <a href="#FUNCTION_FLTBTN_SLOT_5"></a> FUNCTION_FLTBTN_SLOT_5         | `uint8` | 25    |    |
| <a href="#FUNCTION_FLTBTN_SLOT_6"></a> FUNCTION_FLTBTN_SLOT_6         | `uint8` | 26    |    |
| <a href="#FUNCTION_ENGAGE_MAIN_MOTOR"></a> FUNCTION_ENGAGE_MAIN_MOTOR | `uint8` | 27    |    |
| <a href="#FUNCTION_PAYLOAD_POWER"></a> FUNCTION_PAYLOAD_POWER                              | `uint8` | 28    |    |
| <a href="#FUNCTION_TERMINATION"></a> FUNCTION_TERMINATION                                                       | `uint8` | 29    |    |
| <a href="#FUNCTION_FLTBTN_SLOT_COUNT"></a> FUNCTION_FLTBTN_SLOT_COUNT | `uint8` | 6     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RcChannels.msg)

:::details
Click here to see original file

```c
uint64 timestamp						# time since system start (microseconds)

uint8 FUNCTION_THROTTLE      = 0
uint8 FUNCTION_ROLL          = 1
uint8 FUNCTION_PITCH         = 2
uint8 FUNCTION_YAW           = 3
uint8 FUNCTION_RETURN        = 4
uint8 FUNCTION_LOITER        = 5
uint8 FUNCTION_OFFBOARD      = 6
uint8 FUNCTION_FLAPS         = 7
uint8 FUNCTION_AUX_1         = 8
uint8 FUNCTION_AUX_2         = 9
uint8 FUNCTION_AUX_3         = 10
uint8 FUNCTION_AUX_4         = 11
uint8 FUNCTION_AUX_5         = 12
uint8 FUNCTION_AUX_6         = 13
uint8 FUNCTION_PARAM_1       = 14
uint8 FUNCTION_PARAM_2       = 15
uint8 FUNCTION_PARAM_3_5     = 16
uint8 FUNCTION_KILLSWITCH    = 17
uint8 FUNCTION_TRANSITION    = 18
uint8 FUNCTION_GEAR          = 19
uint8 FUNCTION_ARMSWITCH     = 20
uint8 FUNCTION_FLTBTN_SLOT_1 = 21
uint8 FUNCTION_FLTBTN_SLOT_2 = 22
uint8 FUNCTION_FLTBTN_SLOT_3 = 23
uint8 FUNCTION_FLTBTN_SLOT_4 = 24
uint8 FUNCTION_FLTBTN_SLOT_5 = 25
uint8 FUNCTION_FLTBTN_SLOT_6 = 26
uint8 FUNCTION_ENGAGE_MAIN_MOTOR = 27
uint8 FUNCTION_PAYLOAD_POWER = 28
uint8 FUNCTION_TERMINATION = 29

uint8 FUNCTION_FLTBTN_SLOT_COUNT = 6

uint64 timestamp_last_valid					# Timestamp of last valid RC signal
float32[18] channels						# Scaled to -1..1 (throttle: 0..1)
uint8 channel_count						# Number of valid channels
int8[30] function						# Functions mapping
uint8 rssi							# Receive signal strength index
bool signal_lost						# Control signal lost, should be checked together with topic timeout
uint32 frame_drop_count						# Number of dropped frames
```

:::
