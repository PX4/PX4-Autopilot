---
pageClass: is-wide-page
---

# RegisterExtComponentReply (UORB message)

**TOPICS:** register_extcomponent_reply

## Fields

| 参数名                                                                                 | 类型         | Unit [Frame] | Range/Enum | 描述                                                              |
| ----------------------------------------------------------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------- |
| timestamp                                                                           | `uint64`   |                                                                  |            | time since system start (microseconds)       |
| request_id                                                     | `uint64`   |                                                                  |            | ID from the request                                             |
| name                                                                                | `char[25]` |                                                                  |            | name from the request                                           |
| px4_ros2_api_version | `uint16`   |                                                                  |            |                                                                 |
| success                                                                             | `bool`     |                                                                  |            |                                                                 |
| arming_check_id                           | `int8`     |                                                                  |            | arming check registration ID (-1 if invalid) |
| mode_id                                                        | `int8`     |                                                                  |            | assigned mode ID (-1 if invalid)             |
| mode_executor_id                          | `int8`     |                                                                  |            | assigned mode executor ID (-1 if invalid)    |
| not_user_selectable                       | `bool`     |                                                                  |            | mode cannot be selected by the user                             |

## Constants

| 参数名                                                                                         | 类型       | 值 | 描述 |
| ------------------------------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                        | `uint32` | 1 |    |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8`  | 2 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/RegisterExtComponentReply.msg)

:::details
Click here to see original file

```c
uint32 MESSAGE_VERSION = 1

uint64 timestamp # time since system start (microseconds)

uint64 request_id          # ID from the request
char[25] name              # name from the request

uint16 px4_ros2_api_version

bool success
int8 arming_check_id      # arming check registration ID (-1 if invalid)
int8 mode_id              # assigned mode ID (-1 if invalid)
int8 mode_executor_id     # assigned mode executor ID (-1 if invalid)

bool not_user_selectable  # mode cannot be selected by the user

uint8 ORB_QUEUE_LENGTH = 2
```

:::
