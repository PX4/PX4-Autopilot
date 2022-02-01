# GPS position in WGS84 coordinates.
# the field 'timestamp' is for the position & velocity (microseconds)
uint64 timestamp		# time since system start (microseconds)

uint64 timestamp_sample 	# time since system start (microseconds)

uint32 device_id                # unique device ID for the sensor that does not change between power cycles

uint64 time_utc_usec		# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0

uint16 reference_stations_id      # Reference Station ID. Must be in the range 0..4095

float32[3] position               # GPS NED relative position vector (m)
float32[3] position_accuracy      # Accuracy of relative position (m)

float32 heading                   # Heading of the relative position vector (radians)
float32 heading_offset            # Heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
float32 heading_accuracy          # Accuracy of heading of the relative position vector (radians)

float32 position_length
float32 accuracy_length

bool gnss_fix_ok                  # A valid fix (i.e within DOP & accuracy masks)
bool differential_solution        # differential corrections were applied
bool relative_position_valid
bool carrier_solution_floating    # carrier phase range solution with floating ambiguities
bool carrier_solution_fixed       # carrier phase range solution with fixed ambiguities
bool moving_base_mode             # if the receiver is operating in moving base mode
bool reference_position_miss      # extrapolated reference position was used to compute moving base solution this epoch
bool reference_observations_miss  # extrapolated reference observations were used to compute moving base solution this epoch
bool heading_valid
bool relative_position_normalized # the components of the relative position vector (including the high-precision parts) are normalized
