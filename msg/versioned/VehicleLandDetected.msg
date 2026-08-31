uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

bool freefall # true if vehicle is currently in free-fall
bool ground_contact # true if vehicle has ground contact but is not landed (1. stage)
bool maybe_landed # true if the vehicle might have landed (2. stage)
bool landed # true if vehicle is currently landed on the ground (3. stage)

bool in_ground_effect # indicates if from the perspective of the landing detector the vehicle might be in ground effect (baro). This flag will become true if the vehicle is not moving horizontally and is descending (crude assumption that user is landing).

# Flags in this paragraph are for diagnostic logging only
bool in_descend # true only if commanded to descend faster than 1.1 * LNDMC_Z_VEL_MAX for every update since the previous publication
bool has_low_throttle # true only if throttle was at or below the low-throttle threshold for every update since the previous publication
bool vertical_movement # false only if vertical velocity stayed below LNDMC_Z_VEL_MAX for every update since the previous publication
bool horizontal_movement # false only if horizontal velocity stayed below LNDMC_XY_VEL_MAX for every update since the previous publication
bool rotational_movement # false only if roll/pitch rate stayed below LNDMC_ROT_MAX for every update since the previous publication
bool close_to_ground_or_skipped_check # true only if close to the ground, or the check was skipped (no distance sensor), for every update since the previous publication

bool at_rest
