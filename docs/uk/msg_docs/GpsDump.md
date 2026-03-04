---
pageClass: is-wide-page
---

# GpsDump (повідомлення UORB)

This message is used to dump the raw gps communication to the log.

**TOPICS:** gps_dump

## Fields

| Назва                          | Тип         | Unit [Frame] | Range/Enum | Опис                                                      |
| ------------------------------ | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                      | `uint64`    |                                                                  |            | time since system start (microseconds) |
| instance                       | `uint8`     |                                                                  |            | Instance of GNSS receiver                                 |
| device_id | `uint32`    |                                                                  |            |                                                           |
| len                            | `uint8`     |                                                                  |            | length of data, MSB bit set = message to the gps device,  |
| data                           | `uint8[79]` |                                                                  |            | data to write to the log                                  |

## Constants

| Назва                                                                                       | Тип     | Значення | Опис |
| ------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#INSTANCE_MAIN"></a> INSTANCE_MAIN                            | `uint8` | 0        |      |
| <a href="#INSTANCE_SECONDARY"></a> INSTANCE_SECONDARY                  | `uint8` | 1        |      |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 16       |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpsDump.msg)

:::details
Click here to see original file

```c
# This message is used to dump the raw gps communication to the log.

uint64 timestamp # time since system start (microseconds)

uint8 INSTANCE_MAIN = 0
uint8 INSTANCE_SECONDARY = 1

uint8 instance   # Instance of GNSS receiver
uint32 device_id
uint8 len        # length of data, MSB bit set = message to the gps device,
                 # clear = message from the device
uint8[79] data   # data to write to the log

uint8 ORB_QUEUE_LENGTH = 16
```

:::
