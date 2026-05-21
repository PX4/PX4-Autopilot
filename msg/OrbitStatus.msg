# Orbit status
#
# Current state of an orbit or loiter manoeuver, published while the maneuver is executing.
# For multirotors, published by the orbit flight task (FlightTaskOrbit) on each control cycle
# when a valid GPS projection is available.
# For fixed-wing, published by FixedWingModeManager during loiter.
# Subscribed by the MAVLink module and streamed to the GCS as ORBIT_EXECUTION_STATUS (message 360).

uint64 timestamp # [us] Time since system start
float32 radius # [m] Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.

uint8 frame # [@enum FRAME] The coordinate system of the fields: x, y, z
uint8 FRAME_GLOBAL = 0 # WGS84 global frame, MSL altitude. x/y = latitude/longitude (degrees × 1e7)
uint8 FRAME_LOCAL_NED = 1 # Local NED frame. x/y = north/east position (meters × 1e4)
uint8 FRAME_GLOBAL_RELATIVE_ALT = 3 # WGS84 global frame, altitude above home. x/y = latitude/longitude (degrees × 1e7)
uint8 FRAME_GLOBAL_TERRAIN_ALT = 10 # WGS84 global frame, altitude above terrain. x/y = latitude/longitude (degrees × 1e7)

float64 x # X coordinate of center point. Coordinate system depends on frame field: `local = x position in meters * 1e4`, `global = latitude in degrees * 1e7`.
float64 y # Y coordinate of center point. Coordinate system depends on frame field: `local = y position in meters * 1e4`, `global = longitude in degrees * 1e7`.
float32 z # Altitude of center point. Coordinate system depends on frame field.

uint8 yaw_behaviour # [@enum ORBIT_YAW_BEHAVIOUR]
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0 # Vehicle front points to the center (default).
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1 # Vehicle front holds heading when message received.
uint8 ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2 # Yaw uncontrolled.
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3 # Vehicle front follows flight path (tangential to circle).
uint8 ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4 # Yaw controlled by RC input.
uint8 ORBIT_YAW_BEHAVIOUR_UNCHANGED = 5 # Vehicle uses current yaw behaviour (unchanged). The vehicle-default yaw behaviour is used if this value is specified when orbit is first commanded.
