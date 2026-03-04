---
pageClass: is-wide-page
---

# LoggerStatus (UORB message)

**TOPICS:** logger_status

## Fields

| 参数名                                                                            | 类型        | Unit [Frame] | Range/Enum | 描述                                                        |
| ------------------------------------------------------------------------------ | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                                      | `uint64`  |                                                                  |            | time since system start (microseconds) |
| type                                                                           | `uint8`   |                                                                  |            |                                                           |
| backend                                                                        | `uint8`   |                                                                  |            |                                                           |
| is_logging                                                | `bool`    |                                                                  |            |                                                           |
| total_written_kb                     | `float32` |                                                                  |            | total written to log in kiloBytes                         |
| write_rate_kb_s | `float32` |                                                                  |            | write rate in kiloBytes/s                                 |
| dropouts                                                                       | `uint32`  |                                                                  |            | number of failed buffer writes due to buffer overflow     |
| message_gaps                                              | `uint32`  |                                                                  |            | messages misssed                                          |
| buffer_used_bytes                    | `uint32`  |                                                                  |            | current buffer fill in Bytes                              |
| buffer_size_bytes                    | `uint32`  |                                                                  |            | total buffer size in Bytes                                |
| num_messages                                              | `uint8`   |                                                                  |            |                                                           |

## Constants

| 参数名                                                                                               | 类型      | 值 | 描述                                                                                           |
| ------------------------------------------------------------------------------------------------- | ------- | - | -------------------------------------------------------------------------------------------- |
| <a href="#LOGGER_TYPE_FULL"></a> LOGGER_TYPE_FULL       | `uint8` | 0 | Normal, full size log                                                                        |
| <a href="#LOGGER_TYPE_MISSION"></a> LOGGER_TYPE_MISSION | `uint8` | 1 | reduced mission log (e.g. for geotagging) |
| <a href="#BACKEND_FILE"></a> BACKEND_FILE                                    | `uint8` | 1 |                                                                                              |
| <a href="#BACKEND_MAVLINK"></a> BACKEND_MAVLINK                              | `uint8` | 2 |                                                                                              |
| <a href="#BACKEND_ALL"></a> BACKEND_ALL                                      | `uint8` | 3 |                                                                                              |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LoggerStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp               # time since system start (microseconds)

uint8 LOGGER_TYPE_FULL    = 0  # Normal, full size log
uint8 LOGGER_TYPE_MISSION = 1  # reduced mission log (e.g. for geotagging)
uint8 type

uint8 BACKEND_FILE    = 1
uint8 BACKEND_MAVLINK = 2
uint8 BACKEND_ALL     = 3
uint8 backend

bool is_logging

float32 total_written_kb       # total written to log in kiloBytes
float32 write_rate_kb_s        # write rate in kiloBytes/s

uint32 dropouts                # number of failed buffer writes due to buffer overflow
uint32 message_gaps            # messages misssed

uint32 buffer_used_bytes       # current buffer fill in Bytes
uint32 buffer_size_bytes       # total buffer size in Bytes

uint8 num_messages
```

:::
