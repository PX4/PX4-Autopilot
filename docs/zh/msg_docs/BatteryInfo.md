---
pageClass: is-wide-page
---

# BatteryInfo (UORB message)

Battery information.

Static or near-invariant battery information.
Should be streamed at low rate.

**TOPICS:** battery_info

## Fields

| 参数名                                | 类型         | Unit [Frame] | Range/Enum | 描述                                                                                                                            |
| ---------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------- |
| timestamp                          | `uint64`   | us                                                               |            | Time since system start                                                                                                       |
| id                                 | `uint8`    |                                                                  |            | Must match the id in the battery_status message for the same battery                                     |
| serial_number | `char[32]` |                                                                  |            | Serial number of the battery pack in ASCII characters, 0 terminated (Invalid: 0 All bytes) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/BatteryInfo.msg)

:::details
Click here to see original file

```c
# Battery information
#
# Static or near-invariant battery information.
# Should be streamed at low rate.

uint64 timestamp # [us] Time since system start

uint8 id # Must match the id in the battery_status message for the same battery
char[32] serial_number # [@invalid 0 All bytes] Serial number of the battery pack in ASCII characters, 0 terminated
```

:::
