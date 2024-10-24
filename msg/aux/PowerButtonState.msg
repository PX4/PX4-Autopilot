# power button state notification message

uint64 timestamp			    # time since system start (microseconds)

uint8 PWR_BUTTON_STATE_IDEL = 0             # Button went up without meeting shutdown button down time (delete event)
uint8 PWR_BUTTON_STATE_DOWN = 1             # Button went Down
uint8 PWR_BUTTON_STATE_UP = 2               # Button went Up
uint8 PWR_BUTTON_STATE_REQUEST_SHUTDOWN = 3 # Button went Up after meeting shutdown button down time

uint8 event                                 # one of PWR_BUTTON_STATE_*
