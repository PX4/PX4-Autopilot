---
pageClass: is-wide-page
---

# VehicleLandDetected (UORB message)

**TOPICS:** vehicle_land_detected

## Fields

| Name                                                                              | Type     | Unit [Frame] | Range/Enum | Description                                                                                                                                                                                                                             |
| --------------------------------------------------------------------------------- | -------- | ------------ | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                               | `uint64` |              |            | time since system start (microseconds)                                                                                                                                                                                                  |
| <a id="fld_freefall"></a>freefall                                                 | `bool`   |              |            | true if vehicle is currently in free-fall                                                                                                                                                                                               |
| <a id="fld_ground_contact"></a>ground_contact                                     | `bool`   |              |            | true if vehicle has ground contact but is not landed (1. stage)                                                                                                                                                                         |
| <a id="fld_maybe_landed"></a>maybe_landed                                         | `bool`   |              |            | true if the vehicle might have landed (2. stage)                                                                                                                                                                                        |
| <a id="fld_landed"></a>landed                                                     | `bool`   |              |            | true if vehicle is currently landed on the ground (3. stage)                                                                                                                                                                            |
| <a id="fld_in_ground_effect"></a>in_ground_effect                                 | `bool`   |              |            | indicates if from the perspective of the landing detector the vehicle might be in ground effect (baro). This flag will become true if the vehicle is not moving horizontally and is descending (crude assumption that user is landing). |
| <a id="fld_in_descend"></a>in_descend                                             | `bool`   |              |            | true only if commanded to descend faster than 1.1 \* LNDMC_Z_VEL_MAX for every update since the previous publication                                                                                                                    |
| <a id="fld_has_low_throttle"></a>has_low_throttle                                 | `bool`   |              |            | true only if throttle was at or below the low-throttle threshold for every update since the previous publication                                                                                                                        |
| <a id="fld_vertical_movement"></a>vertical_movement                               | `bool`   |              |            | false only if vertical velocity stayed below LNDMC_Z_VEL_MAX for every update since the previous publication                                                                                                                            |
| <a id="fld_horizontal_movement"></a>horizontal_movement                           | `bool`   |              |            | false only if horizontal velocity stayed below LNDMC_XY_VEL_MAX for every update since the previous publication                                                                                                                         |
| <a id="fld_rotational_movement"></a>rotational_movement                           | `bool`   |              |            | false only if roll/pitch rate stayed below LNDMC_ROT_MAX for every update since the previous publication                                                                                                                                |
| <a id="fld_close_to_ground_or_skipped_check"></a>close_to_ground_or_skipped_check | `bool`   |              |            | true only if close to the ground, or the check was skipped (no distance sensor), for every update since the previous publication                                                                                                        |
| <a id="fld_at_rest"></a>at_rest                                                   | `bool`   |              |            |

## Constants

| Name                                          | Type     | Value | Description |
| --------------------------------------------- | -------- | ----- | ----------- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleLandDetected.msg)

::: details Click here to see original file

```c
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
```

:::
