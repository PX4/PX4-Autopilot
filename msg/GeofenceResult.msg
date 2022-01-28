uint64 timestamp                # time since system start (microseconds)
uint8 GF_ACTION_NONE      = 0   # no action on geofence violation
uint8 GF_ACTION_WARN      = 1   # critical mavlink message
uint8 GF_ACTION_LOITER    = 2   # switch to AUTO|LOITER
uint8 GF_ACTION_RTL       = 3   # switch to AUTO|RTL
uint8 GF_ACTION_TERMINATE = 4   # flight termination
uint8 GF_ACTION_LAND      = 5   # switch to AUTO|LAND

bool max_altitude_exceeded       # true if the maximum altitude allowed is exceeded
bool max_distance_exceeded       # true if the maximum distance from home allowed is exceeded
bool primary_geofence_breached   # true if the primary geofence is breached
bool secondary_geofence_breached # true if the secondary geofence is breached

uint8 primary_geofence_action    # action to take when the primary geofence is breached
uint8 secondary_geofence_action  # action to take when the secondary geofence is breached

bool home_required               # true if the geofence requires a valid home position
