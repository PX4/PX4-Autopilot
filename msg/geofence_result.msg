uint64 timestamp                # time since system start (microseconds)
uint8 GF_ACTION_DEFAULT = 0     # use action from parameter
uint8 GF_ACTION_NONE = 1        # no action on geofence violation
uint8 GF_ACTION_WARN = 2        # critical mavlink message
uint8 GF_ACTION_LOITER = 3      # switch to AUTO|LOITER (Hold mode)
uint8 GF_ACTION_RTL = 4         # switch to AUTO|RTL
uint8 GF_ACTION_LAND = 5        # switch to AUTO|LAND
uint8 GF_ACTION_TERMINATE = 6   # flight termination

bool geofence_violated          # true if the geofence is violated
uint8 geofence_action           # action to take when geofence is violated
bool action_required            # fence with action with different than GF_ACTION_NONE exist

bool home_required              # true if the geofence requires a valid home position
