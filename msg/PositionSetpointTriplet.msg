# Global position setpoint triplet in WGS84 coordinates.
# This are the three next waypoints (or just the next two or one).

uint64 timestamp		# time since system start (microseconds)

PositionSetpoint previous
PositionSetpoint current
PositionSetpoint next
PositionSetpoint next_next	# waypoint after next, only used as lookahead for the trajectory speed planning (valid=false if unknown)
