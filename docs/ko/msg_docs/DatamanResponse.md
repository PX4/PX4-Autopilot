# DatamanResponse (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DatamanResponse.msg)

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
