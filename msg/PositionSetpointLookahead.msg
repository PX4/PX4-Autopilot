# Waypoint beyond the next position setpoint, published by the navigator purely as a lookahead
# for multicopter trajectory speed planning.
#
# This is deliberately not part of PositionSetpointTriplet: it is not a navigation target and no
# position controller is allowed to fly to it. It only tells the trajectory generator which geometry
# follows the next waypoint, so that the speed which may be carried through the next waypoint can be
# derived from the real turn angle and leg length instead of assuming a full stop there.
#
# valid is false whenever the geometry after the next waypoint is unknown, or whenever the vehicle is
# expected to come to a stop at the next waypoint anyway (hold time, timeout, no autocontinue, land,
# loiter, VTOL transition, end of mission). Consumers must then fall back to assuming a full stop at
# the next waypoint.

uint64 timestamp		# time since system start (microseconds)

bool valid			# true if lat/lon/alt describe a known waypoint after the next one

float64 lat			# latitude, in deg
float64 lon			# longitude, in deg
float32 alt			# altitude AMSL, in m
