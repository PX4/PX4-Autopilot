uint64 timestamp # time since system start [us]

uint8 id # Must match the id in the battery_status message for the same battery
char[32] serial_number # Serial number of the battery pack in ASCII characters, 0 terminated [@invalid All bytes 0]
