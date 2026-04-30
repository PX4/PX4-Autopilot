# Orbit status
#
# Current state of an orbit or loiter maneuver, published while the maneuver is executing.
# For multirotors, published by the orbit flight task (FlightTaskOrbit) on each control cycle
# when a valid GPS projection is available.
# For fixed-wing, published by FixedWingModeManager during loiter.
# Subscribed by the MAVLink module and streamed to the GCS as ORBIT_EXECUTION_STATUS (message 360).

uint64 timestamp # [us] Time since system start
float32 radius # [m] Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.
uint8 frame # The coordinate system of the fields: x, y, z.
float64 x # X coordinate of center point. Coordinate system depends on frame field: `local = x position in meters * 1e4`, `global = latitude in degrees * 1e7`.
float64 y # Y coordinate of center point. Coordinate system depends on frame field: `local = y position in meters * 1e4`, `global = longitude in degrees * 1e7`.
float32 z # Altitude of center point. Coordinate system depends on frame field.

uint8 yaw_behaviour # [@enum ORBIT_YAW_BEHAVIOUR]
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1
uint8 ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3
uint8 ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4
uint8 ORBIT_YAW_BEHAVIOUR_UNCHANGED = 5
