# Current status of a Navigator mode
# The possible values of nav_state are defined in the VehicleStatus msg.
uint64 timestamp  # time since system start (microseconds)

uint8 nav_state   # Source mode (values in VehicleStatus)
bool failure      # True when the current mode cannot continue
