---
pageClass: is-wide-page
---

# DatamanResponse (повідомлення UORB)

**TOPICS:** dataman_response

## Fields

| Назва                             | Тип         | Unit [Frame] | Range/Enum | Опис                                                      |
| --------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                         | `uint64`    |                                                                  |            | time since system start (microseconds) |
| client_id    | `uint8`     |                                                                  |            |                                                           |
| request_type | `uint8`     |                                                                  |            | id/read/write/clear                                       |
| item                              | `uint8`     |                                                                  |            | dm_item_t       |
| index                             | `uint32`    |                                                                  |            |                                                           |
| data                              | `uint8[56]` |                                                                  |            |                                                           |
| status                            | `uint8`     |                                                                  |            |                                                           |

## Constants

| Назва                                                                                                                                  | Тип     | Значення | Опис |
| -------------------------------------------------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#STATUS_SUCCESS"></a> STATUS_SUCCESS                                                                     | `uint8` | 0        |      |
| <a href="#STATUS_FAILURE_ID_ERR"></a> STATUS_FAILURE_ID_ERR             | `uint8` | 1        |      |
| <a href="#STATUS_FAILURE_NO_DATA"></a> STATUS_FAILURE_NO_DATA           | `uint8` | 2        |      |
| <a href="#STATUS_FAILURE_READ_FAILED"></a> STATUS_FAILURE_READ_FAILED   | `uint8` | 3        |      |
| <a href="#STATUS_FAILURE_WRITE_FAILED"></a> STATUS_FAILURE_WRITE_FAILED | `uint8` | 4        |      |
| <a href="#STATUS_FAILURE_CLEAR_FAILED"></a> STATUS_FAILURE_CLEAR_FAILED | `uint8` | 5        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DatamanResponse.msg)

:::details
Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)

uint8 client_id
uint8 request_type	# id/read/write/clear
uint8 item			# dm_item_t
uint32 index
uint8[56] data

uint8 STATUS_SUCCESS = 0
uint8 STATUS_FAILURE_ID_ERR = 1
uint8 STATUS_FAILURE_NO_DATA = 2
uint8 STATUS_FAILURE_READ_FAILED = 3
uint8 STATUS_FAILURE_WRITE_FAILED = 4
uint8 STATUS_FAILURE_CLEAR_FAILED = 5
uint8 status
```

:::
