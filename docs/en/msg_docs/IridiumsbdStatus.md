---
pageClass: is-wide-page
---

# IridiumsbdStatus (UORB message)

**TOPICS:** iridiumsbd_status

## Fields

| Name                                                            | Type     | Unit [Frame] | Range/Enum | Description                                                                          |
| --------------------------------------------------------------- | -------- | ------------ | ---------- | ------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                             | `uint64` |              |            | time since system start (microseconds)                                               |
| <a id="fld_last_at_ok_timestamp"></a>last_at_ok_timestamp       | `uint64` |              |            | timestamp of the last "OK" received after the "AT" command                           |
| <a id="fld_tx_buf_write_index"></a>tx_buf_write_index           | `uint16` |              |            | current size of the tx buffer                                                        |
| <a id="fld_rx_buf_read_index"></a>rx_buf_read_index             | `uint16` |              |            | the rx buffer is parsed up to that index                                             |
| <a id="fld_rx_buf_end_index"></a>rx_buf_end_index               | `uint16` |              |            | current size of the rx buffer                                                        |
| <a id="fld_failed_sbd_sessions"></a>failed_sbd_sessions         | `uint16` |              |            | number of failed sbd sessions                                                        |
| <a id="fld_successful_sbd_sessions"></a>successful_sbd_sessions | `uint16` |              |            | number of successful sbd sessions                                                    |
| <a id="fld_num_tx_buf_reset"></a>num_tx_buf_reset               | `uint16` |              |            | number of times the tx buffer was reset                                              |
| <a id="fld_signal_quality"></a>signal_quality                   | `uint8`  |              |            | current signal quality, 0 is no signal, 5 the best                                   |
| <a id="fld_state"></a>state                                     | `uint8`  |              |            | current state of the driver, see the satcom_state of IridiumSBD.h for the definition |
| <a id="fld_ring_pending"></a>ring_pending                       | `bool`   |              |            | indicates if a ring call is pending                                                  |
| <a id="fld_tx_buf_write_pending"></a>tx_buf_write_pending       | `bool`   |              |            | indicates if a tx buffer write is pending                                            |
| <a id="fld_tx_session_pending"></a>tx_session_pending           | `bool`   |              |            | indicates if a tx session is pending                                                 |
| <a id="fld_rx_read_pending"></a>rx_read_pending                 | `bool`   |              |            | indicates if a rx read is pending                                                    |
| <a id="fld_rx_session_pending"></a>rx_session_pending           | `bool`   |              |            | indicates if a rx session is pending                                                 |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/IridiumsbdStatus.msg)

::: details Click here to see original file

```c
uint64 timestamp				# time since system start (microseconds)
uint64 last_at_ok_timestamp			# timestamp of the last "OK" received after the "AT" command
uint16 tx_buf_write_index			# current size of the tx buffer
uint16 rx_buf_read_index			# the rx buffer is parsed up to that index
uint16 rx_buf_end_index				# current size of the rx buffer
uint16 failed_sbd_sessions			# number of failed sbd sessions
uint16 successful_sbd_sessions      # number of successful sbd sessions
uint16 num_tx_buf_reset             # number of times the tx buffer was reset
uint8 signal_quality                # current signal quality, 0 is no signal, 5 the best
uint8 state                         # current state of the driver, see the satcom_state of IridiumSBD.h for the definition
bool ring_pending                   # indicates if a ring call is pending
bool tx_buf_write_pending           # indicates if a tx buffer write is pending
bool tx_session_pending             # indicates if a tx session is pending
bool rx_read_pending                # indicates if a rx read is pending
bool rx_session_pending             # indicates if a rx session is pending
```

:::
