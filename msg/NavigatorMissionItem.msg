uint64 timestamp                 # time since system start (microseconds)

uint32 instance_count            # Instance count of this mission. Increments monotonically whenever the mission is modified

uint16 sequence_current          # Sequence of the current mission item

uint16 nav_cmd

# Work Item corresponds to the sub-mode set on the "MAV_CMD_DO_SET_MODE" MAVLink message
uint8 WORK_ITEM_TYPE_DEFAULT = 0                        # Default mission item
uint8 WORK_ITEM_TYPE_TAKEOFF = 1                        # Takeoff before moving to waypoint
uint8 WORK_ITEM_TYPE_MOVE_TO_LAND = 2                   # Move to land waypoint before descent
uint8 WORK_ITEM_TYPE_ALIGN = 3                          # Align for next waypoint
uint8 WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF = 4        # VTOL specific
uint8 WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION = 5  # VTOL specific
uint8 WORK_ITEM_TYPE_PRECISION_LAND = 6                 # Precision Landing for Multicopters using external aid

uint8 nav_sub_cmd                # Synonymous to "work_item_type" in Navigator. Will be renamed eventually in navigator.
                                 # sub-command, for example AUTO_PRECLAND in main command AUTO_MISSION

float32 latitude
float32 longitude

float32 time_inside              # time that the MAV should stay inside the radius before advancing in seconds
float32 acceptance_radius        # default radius in which the mission is accepted as reached in meters
float32 loiter_radius            # loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise
float32 yaw                      # in radians NED -PI..+PI, NAN means don't change yaw
float32 altitude                 # altitude in meters (AMSL)

uint8 frame                      # mission frame
uint8 origin                     # mission item origin (onboard or mavlink)

bool loiter_exit_xtrack          # exit xtrack location: 0 for center of loiter wp, 1 for exit location
bool force_heading               # heading needs to be reached
bool altitude_is_relative        # true if altitude is relative from start point
bool autocontinue                # true if next waypoint should follow after this one
bool vtol_back_transition        # part of the vtol back transition sequence
