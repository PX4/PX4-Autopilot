# Detect-and-avoid conflict assessment for one traffic aircraft.
#
# This is the detailed per-traffic output from navigator's `DetectAndAvoid` component.
# A sample is published for each processed `transponder_report` with a usable
# traffic identifier and valid data.
#
# Unlike `detect_and_avoid_most_urgent`, this topic does not summarize the single
# active conflict driving DAA actions or prearm checks. It reports the conflict level
# and geometry for the specific traffic aircraft that was just evaluated.
#
# Published by: `navigator` (`DetectAndAvoid`)
# Used by: logging, and test.

uint8 DAA_CONFLICT_LVL_NONE = 0
uint8 DAA_CONFLICT_LVL_LOW = 1
uint8 DAA_CONFLICT_LVL_MEDIUM = 2
uint8 DAA_CONFLICT_LVL_HIGH = 3
uint8 DAA_CONFLICT_LVL_CRITICAL = 4

uint8 DAA_STANDARD_CROSSTRACK = 0
uint8 DAA_STANDARD_F3442 = 1 # ASTM F3442/F3442M-based conflict evaluation

uint8 UNIQUE_ID_ENCODING_ICAO = 0 # `unique_id` contains an ICAO address
uint8 UNIQUE_ID_ENCODING_ADSB_CALLSIGN = 1 # `unique_id` contains an ADS-B callsign packed into a `uint64`
uint8 UNIQUE_ID_ENCODING_UAS_ID = 2 # `unique_id` contains the reduced tail bytes of a UAS ID

uint64 timestamp # [us] Time since system start
uint64 unique_id # [-] Encoded traffic identifier selected in priority order: ICAO address, ADS-B callsign, then reduced UAS ID tail bytes
uint8 unique_id_encoding # [-] Namespace used to decode `unique_id`

uint8 conflict_level # [-] Conflict level calculated for this traffic aircraft
float32 aircraft_dist_hor # [m] [@frame NED] Horizontal separation metric. In Crosstrack mode this is the signed crosstrack distance when available, otherwise direct horizontal range
float32 aircraft_dist_vert # [m] [@frame NED] Vertical separation between ownship (the current vehicle) and the traffic aircraft
float32 expected_min_dist_time # [s] Estimated time until minimum separation between ownship (the current vehicle) and the traffic aircraft
