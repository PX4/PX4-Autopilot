# External acceleration setpoint in NED frame
# Used to inject acceleration commands into Position/Altitude mode
# without requiring Offboard mode.
# Set type_mask bit 11 in SET_POSITION_TARGET_LOCAL_NED to publish this.
# NaN for yaw means hold current yaw.

uint64 timestamp        # time since system start (microseconds)

float32[3] acceleration # NED frame, in meters/second^2
float32 yaw             # euler angle in radians -PI..+PI, NaN = hold current
uint16 timeout_ms       # watchdog: revert to hover if no update within this period (ms)
