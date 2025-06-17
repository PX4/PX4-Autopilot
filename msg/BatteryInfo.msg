# Battery information
#
# Static or near-invariant battery information.
# Should be streamed at low rate.

uint64 timestamp # [us] Time since system start

uint8 id # Must match the id in the battery_status message for the same battery
char[32] serial_number # [@invalid 0 All bytes] Serial number of the battery pack in ASCII characters, 0 terminated
