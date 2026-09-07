# Status of a fixed-wing takeoff
# Passes information from the FixedWingModeManager to the Navigator.

uint64 timestamp # [us] time since system start

bool climbout_completed  # Whether the takeoff climbout is finished (altitude, or time if FW_TKO_CLMB_T is set)
