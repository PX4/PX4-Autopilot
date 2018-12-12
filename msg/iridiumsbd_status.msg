uint64 timestamp				# time since system start (microseconds)
uint64 last_heartbeat				# timestamp of the last successful sbd session
uint16 tx_buf_write_index			# current size of the tx buffer
uint16 rx_buf_read_index			# the rx buffer is parsed up to that index
uint16 rx_buf_end_index				# current size of the rx buffer
uint16 failed_sbd_sessions			# number of failed sbd sessions
uint16 successful_sbd_sessions      # number of successfull sbd sessions
uint16 num_tx_buf_reset             # number of times the tx buffer was reset
uint8 signal_quality                # current signal quality, 0 is no signal, 5 the best
uint8 state                         # current state of the driver, see the satcom_state of IridiumSBD.h for the definition
bool ring_pending                   # indicates if a ring call is pending
bool tx_buf_write_pending           # indicates if a tx buffer write is pending
bool tx_session_pending             # indicates if a tx session is pending
bool rx_read_pending                # indicates if a rx read is pending
bool rx_session_pending             # indicates if a rx session is pending
