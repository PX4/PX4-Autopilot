---
pageClass: is-wide-page
---

# DatamanRequest (UORB message)

**TOPICS:** dataman_request

## Fields

| Name                                      | Type        | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------------- | ----------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp       | `uint64`    |              |            | time since system start (microseconds) |
| <a id="fld_client_id"></a>client_id       | `uint8`     |              |            |
| <a id="fld_request_type"></a>request_type | `uint8`     |              |            | id/read/write/clear                    |
| <a id="fld_item"></a>item                 | `uint8`     |              |            | dm_item_t                              |
| <a id="fld_index"></a>index               | `uint32`    |              |            |
| <a id="fld_data"></a>data                 | `uint8[56]` |              |            |
| <a id="fld_data_length"></a>data_length   | `uint32`    |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DatamanRequest.msg)

::: details Click here to see original file

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
