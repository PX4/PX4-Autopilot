---
pageClass: is-wide-page
---

# DatamanResponse (UORB message)

**TOPICS:** dataman_response

## Fields

| Name                                      | Type        | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------------- | ----------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp       | `uint64`    |              |            | time since system start (microseconds) |
| <a id="fld_client_id"></a>client_id       | `uint8`     |              |            |
| <a id="fld_request_type"></a>request_type | `uint8`     |              |            | id/read/write/clear                    |
| <a id="fld_item"></a>item                 | `uint8`     |              |            | dm_item_t                              |
| <a id="fld_index"></a>index               | `uint32`    |              |            |
| <a id="fld_data"></a>data                 | `uint8[56]` |              |            |
| <a id="fld_status"></a>status             | `uint8`     |              |            |

## Constants

| Name                                                                  | Type    | Value | Description |
| --------------------------------------------------------------------- | ------- | ----- | ----------- |
| <a id="#STATUS_SUCCESS"></a> STATUS_SUCCESS                           | `uint8` | 0     |
| <a id="#STATUS_FAILURE_ID_ERR"></a> STATUS_FAILURE_ID_ERR             | `uint8` | 1     |
| <a id="#STATUS_FAILURE_NO_DATA"></a> STATUS_FAILURE_NO_DATA           | `uint8` | 2     |
| <a id="#STATUS_FAILURE_READ_FAILED"></a> STATUS_FAILURE_READ_FAILED   | `uint8` | 3     |
| <a id="#STATUS_FAILURE_WRITE_FAILED"></a> STATUS_FAILURE_WRITE_FAILED | `uint8` | 4     |
| <a id="#STATUS_FAILURE_CLEAR_FAILED"></a> STATUS_FAILURE_CLEAR_FAILED | `uint8` | 5     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DatamanResponse.msg)

::: details Click here to see original file

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
