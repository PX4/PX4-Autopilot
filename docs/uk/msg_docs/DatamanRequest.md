---
pageClass: is-wide-page
---

# DatamanRequest (повідомлення UORB)

**TOPICS:** dataman_request

## Fields

| Назва                             | Тип         | Unit [Frame] | Range/Enum | Опис                                                      |
| --------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                         | `uint64`    |                                                                  |            | time since system start (microseconds) |
| client_id    | `uint8`     |                                                                  |            |                                                           |
| request_type | `uint8`     |                                                                  |            | id/read/write/clear                                       |
| item                              | `uint8`     |                                                                  |            | dm_item_t       |
| index                             | `uint32`    |                                                                  |            |                                                           |
| data                              | `uint8[56]` |                                                                  |            |                                                           |
| data_length  | `uint32`    |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DatamanRequest.msg)

:::details
Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)

uint8 client_id
uint8 request_type	# id/read/write/clear
uint8 item			# dm_item_t
uint32 index
uint8[56] data
uint32 data_length
```

:::
