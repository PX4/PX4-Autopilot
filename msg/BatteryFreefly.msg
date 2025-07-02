# Freefly specific smart battery information
uint64 timestamp # time since system start (microseconds)

uint8 id # Must match the id in the battery_status message for the same battery
uint8[2] firmware_version # [0] - major, [1] - minor
uint8 model # Battery model
uint32 custom_faults # Bitmask indicating smart battery internal manufacturer faults, those are not user actionable.

uint8 mode # Battery mode. Note, the normal operation mode
uint8 BATTERY_MODE_HOTSWAP = 7 # Powering vehicle limited load for idling and hot swapping
uint8 BATTERY_MODE_AIRMODE = 8 # Armed for flight and does not limit load
