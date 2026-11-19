# Navigator mission item
#
# The mission item that navigator is currently executing, for example while flying a mission or a mission-based RTL.
# Published by `navigator` whenever it starts a new item, for logging.
# Used by `vision_target_estimator` to get the position of the current landing item.

uint64 timestamp # [us] Time since system start

uint16 sequence_current # [-] Sequence number of the current mission item

uint16 nav_cmd # [-] Navigation command, as defined by MAVLink MAV_CMD

float32 latitude # [deg] Latitude
float32 longitude # [deg] Longitude

float32 time_inside # [s] Time the vehicle has to stay inside the acceptance radius before the item is considered reached
float32 acceptance_radius # [m] Radius around the item position within which the item is considered reached
float32 loiter_radius # [m] Loiter radius. 0 for a VTOL to hover, negative for counter-clockwise
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN to not change yaw] Yaw setpoint
float32 altitude # [m] Altitude, referenced as defined by `frame`

uint8 frame # [@enum FRAME] Frame of the item, a subset of MAVLink MAV_FRAME. Defines how `altitude` is referenced
uint8 FRAME_GLOBAL = 0 # Altitude above mean sea level (AMSL)
uint8 FRAME_MISSION = 2 # Item without a position, such as a DO command. `latitude`, `longitude` and `altitude` are unused
uint8 FRAME_GLOBAL_RELATIVE_ALT = 3 # Altitude relative to the home position

uint8 origin # [@enum ORIGIN] Where the mission item was generated
uint8 ORIGIN_MAVLINK = 0 # Uploaded through MAVLink
uint8 ORIGIN_ONBOARD = 1 # Generated onboard by navigator

bool loiter_exit_xtrack # Loiter exit location. False: from the center of the loiter, true: from the exit location
bool force_heading # True if the heading needs to be reached before the item is considered reached
bool autocontinue # True if the next mission item should follow after this one
bool vtol_back_transition # True if the item is part of a VTOL back transition sequence
