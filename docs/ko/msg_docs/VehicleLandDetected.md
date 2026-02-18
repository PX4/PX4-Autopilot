---
pageClass: is-wide-page
---

# VehicleLandDetected (UORB message)

**TOPICS:** vehicle_landdetected

## Fields

| 명칭                                                                                                                                        | 형식       | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                                                                                                                                                            |
| ----------------------------------------------------------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                                                                 | `uint64` |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                                                                                     |
| freefall                                                                                                                                  | `bool`   |                                                                  |            | true if vehicle is currently in free-fall                                                                                                                                                                                                                                                                     |
| ground_contact                                                                                                       | `bool`   |                                                                  |            | true if vehicle has ground contact but is not landed (1. stage)                                                                                                                                                                                                            |
| maybe_landed                                                                                                         | `bool`   |                                                                  |            | true if the vehicle might have landed (2. stage)                                                                                                                                                                                                                           |
| landed                                                                                                                                    | `bool`   |                                                                  |            | true if vehicle is currently landed on the ground (3. stage)                                                                                                                                                                                                               |
| in_ground_effect                                                                                | `bool`   |                                                                  |            | indicates if from the perspective of the landing detector the vehicle might be in ground effect (baro). This flag will become true if the vehicle is not moving horizontally and is descending (crude assumption that user is landing). |
| in_descend                                                                                                           | `bool`   |                                                                  |            |                                                                                                                                                                                                                                                                                                               |
| has_low_throttle                                                                                | `bool`   |                                                                  |            |                                                                                                                                                                                                                                                                                                               |
| vertical_movement                                                                                                    | `bool`   |                                                                  |            |                                                                                                                                                                                                                                                                                                               |
| horizontal_movement                                                                                                  | `bool`   |                                                                  |            |                                                                                                                                                                                                                                                                                                               |
| rotational_movement                                                                                                  | `bool`   |                                                                  |            |                                                                                                                                                                                                                                                                                                               |
| close_to_ground_or_skipped_check | `bool`   |                                                                  |            |                                                                                                                                                                                                                                                                                                               |
| at_rest                                                                                                              | `bool`   |                                                                  |            |                                                                                                                                                                                                                                                                                                               |

## Constants

| 명칭                                                                   | 형식       | Value | 설명 |
| -------------------------------------------------------------------- | -------- | ----- | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleLandDetected.msg)

:::details
Click here to see original file

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp	# time since system start (microseconds)

bool freefall		# true if vehicle is currently in free-fall
bool ground_contact	# true if vehicle has ground contact but is not landed (1. stage)
bool maybe_landed	# true if the vehicle might have landed (2. stage)
bool landed		# true if vehicle is currently landed on the ground (3. stage)

bool in_ground_effect # indicates if from the perspective of the landing detector the vehicle might be in ground effect (baro). This flag will become true if the vehicle is not moving horizontally and is descending (crude assumption that user is landing).
bool in_descend

bool has_low_throttle

bool vertical_movement
bool horizontal_movement
bool rotational_movement

bool close_to_ground_or_skipped_check

bool at_rest
```

:::
