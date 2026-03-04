---
pageClass: is-wide-page
---

# TuneControl (UORB message)

This message is used to control the tunes, when the tune_id is set to CUSTOM. then the frequency, duration are used otherwise those values are ignored.

**TOPICS:** tune_control

## Fields

| 参数名                                | 类型       | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                          |
| ---------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                          | `uint64` |                                                                  |            | time since system start (microseconds)                                                                                                                   |
| tune_id       | `uint8`  |                                                                  |            | tune_id corresponding to TuneID::\* from the tune_defaults.h in the tunes library |
| tune_override | `bool`   |                                                                  |            | if true the tune which is playing will be stopped and the new started                                                                                                       |
| frequency                          | `uint16` |                                                                  |            | in Hz                                                                                                                                                                       |
| duration                           | `uint32` |                                                                  |            | in us                                                                                                                                                                       |
| silence                            | `uint32` |                                                                  |            | in us                                                                                                                                                                       |
| volume                             | `uint8`  |                                                                  |            | value between 0-100 if supported by backend                                                                                                                                 |

## Constants

| 参数名                                                                                                                                                           | 类型      | 值   | 描述 |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | --- | -- |
| <a href="#TUNE_ID_STOP"></a> TUNE_ID_STOP                                                                           | `uint8` | 0   |    |
| <a href="#TUNE_ID_STARTUP"></a> TUNE_ID_STARTUP                                                                     | `uint8` | 1   |    |
| <a href="#TUNE_ID_ERROR"></a> TUNE_ID_ERROR                                                                         | `uint8` | 2   |    |
| <a href="#TUNE_ID_NOTIFY_POSITIVE"></a> TUNE_ID_NOTIFY_POSITIVE                                | `uint8` | 3   |    |
| <a href="#TUNE_ID_NOTIFY_NEUTRAL"></a> TUNE_ID_NOTIFY_NEUTRAL                                  | `uint8` | 4   |    |
| <a href="#TUNE_ID_NOTIFY_NEGATIVE"></a> TUNE_ID_NOTIFY_NEGATIVE                                | `uint8` | 5   |    |
| <a href="#TUNE_ID_ARMING_WARNING"></a> TUNE_ID_ARMING_WARNING                                  | `uint8` | 6   |    |
| <a href="#TUNE_ID_BATTERY_WARNING_SLOW"></a> TUNE_ID_BATTERY_WARNING_SLOW | `uint8` | 7   |    |
| <a href="#TUNE_ID_BATTERY_WARNING_FAST"></a> TUNE_ID_BATTERY_WARNING_FAST | `uint8` | 8   |    |
| <a href="#TUNE_ID_GPS_WARNING"></a> TUNE_ID_GPS_WARNING                                        | `uint8` | 9   |    |
| <a href="#TUNE_ID_ARMING_FAILURE"></a> TUNE_ID_ARMING_FAILURE                                  | `uint8` | 10  |    |
| <a href="#TUNE_ID_PARACHUTE_RELEASE"></a> TUNE_ID_PARACHUTE_RELEASE                            | `uint8` | 11  |    |
| <a href="#TUNE_ID_SINGLE_BEEP"></a> TUNE_ID_SINGLE_BEEP                                        | `uint8` | 12  |    |
| <a href="#TUNE_ID_HOME_SET"></a> TUNE_ID_HOME_SET                                              | `uint8` | 13  |    |
| <a href="#TUNE_ID_SD_INIT"></a> TUNE_ID_SD_INIT                                                | `uint8` | 14  |    |
| <a href="#TUNE_ID_SD_ERROR"></a> TUNE_ID_SD_ERROR                                              | `uint8` | 15  |    |
| <a href="#TUNE_ID_PROG_PX4IO"></a> TUNE_ID_PROG_PX4IO                                          | `uint8` | 16  |    |
| <a href="#TUNE_ID_PROG_PX4IO_OK"></a> TUNE_ID_PROG_PX4IO_OK               | `uint8` | 17  |    |
| <a href="#TUNE_ID_PROG_PX4IO_ERR"></a> TUNE_ID_PROG_PX4IO_ERR             | `uint8` | 18  |    |
| <a href="#TUNE_ID_POWER_OFF"></a> TUNE_ID_POWER_OFF                                            | `uint8` | 19  |    |
| <a href="#NUMBER_OF_TUNES"></a> NUMBER_OF_TUNES                                                                     | `uint8` | 20  |    |
| <a href="#VOLUME_LEVEL_MIN"></a> VOLUME_LEVEL_MIN                                                                   | `uint8` | 0   |    |
| <a href="#VOLUME_LEVEL_DEFAULT"></a> VOLUME_LEVEL_DEFAULT                                                           | `uint8` | 20  |    |
| <a href="#VOLUME_LEVEL_MAX"></a> VOLUME_LEVEL_MAX                                                                   | `uint8` | 100 |    |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                                                   | `uint8` | 4   |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TuneControl.msg)

:::details
Click here to see original file

```c
# This message is used to control the tunes, when the tune_id is set to CUSTOM
# then the frequency, duration are used otherwise those values are ignored.

uint64 timestamp     # time since system start (microseconds)

uint8 TUNE_ID_STOP                 = 0
uint8 TUNE_ID_STARTUP              = 1
uint8 TUNE_ID_ERROR                = 2
uint8 TUNE_ID_NOTIFY_POSITIVE      = 3
uint8 TUNE_ID_NOTIFY_NEUTRAL       = 4
uint8 TUNE_ID_NOTIFY_NEGATIVE      = 5
uint8 TUNE_ID_ARMING_WARNING       = 6
uint8 TUNE_ID_BATTERY_WARNING_SLOW = 7
uint8 TUNE_ID_BATTERY_WARNING_FAST = 8
uint8 TUNE_ID_GPS_WARNING          = 9
uint8 TUNE_ID_ARMING_FAILURE       = 10
uint8 TUNE_ID_PARACHUTE_RELEASE    = 11
uint8 TUNE_ID_SINGLE_BEEP          = 12
uint8 TUNE_ID_HOME_SET             = 13
uint8 TUNE_ID_SD_INIT              = 14
uint8 TUNE_ID_SD_ERROR             = 15
uint8 TUNE_ID_PROG_PX4IO           = 16
uint8 TUNE_ID_PROG_PX4IO_OK        = 17
uint8 TUNE_ID_PROG_PX4IO_ERR       = 18
uint8 TUNE_ID_POWER_OFF            = 19
uint8 NUMBER_OF_TUNES              = 20

uint8 tune_id        # tune_id corresponding to TuneID::* from the tune_defaults.h in the tunes library
bool tune_override   # if true the tune which is playing will be stopped and the new started
uint16 frequency     # in Hz
uint32 duration      # in us
uint32 silence       # in us
uint8 volume         # value between 0-100 if supported by backend

uint8 VOLUME_LEVEL_MIN = 0
uint8 VOLUME_LEVEL_DEFAULT = 20
uint8 VOLUME_LEVEL_MAX = 100

uint8 ORB_QUEUE_LENGTH = 4
```

:::
