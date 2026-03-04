---
pageClass: is-wide-page
---

# VehicleGlobalPosition (повідомлення UORB)

Об'єднана глобальна позиція в WGS84. This struct contains global position estimation. It is not the raw GPS. measurement (@see vehicle_gps_position). This topic is usually published by the position. estimator, which will take more sources of information into account than just GPS,. e.g. control inputs of the vehicle in a Kalman-filter implementation.

**TOPICS:** vehicle_global_position vehicle_global_position_groundtruth external_ins_global_position estimator_global_position aux_global_position

## Fields

| Назва                                                                                | Тип       | Unit [Frame] | Range/Enum | Опис                                                                         |
| ------------------------------------------------------------------------------------ | --------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------- |
| timestamp                                                                            | `uint64`  |                                                                  |            | time since system start (microseconds)                    |
| timestamp_sample                                                | `uint64`  |                                                                  |            | the timestamp of the raw data (microseconds)              |
| lat                                                                                  | `float64` |                                                                  |            | Latitude, (degrees)                                       |
| lon                                                                                  | `float64` |                                                                  |            | Longitude, (degrees)                                      |
| alt                                                                                  | `float32` |                                                                  |            | Altitude AMSL, (meters)                                   |
| alt_ellipsoid                                                   | `float32` |                                                                  |            | Altitude above ellipsoid, (meters)                        |
| lat_lon_valid                              | `bool`    |                                                                  |            |                                                                              |
| alt_valid                                                       | `bool`    |                                                                  |            |                                                                              |
| delta_alt                                                       | `float32` |                                                                  |            | Reset delta for altitude                                                     |
| delta_terrain                                                   | `float32` |                                                                  |            | Reset delta for terrain                                                      |
| lat_lon_reset_counter | `uint8`   |                                                                  |            | Counter for reset events on horizontal position coordinates                  |
| alt_reset_counter                          | `uint8`   |                                                                  |            | Counter for reset events on altitude                                         |
| terrain_reset_counter                      | `uint8`   |                                                                  |            | Counter for reset events on terrain                                          |
| eph                                                                                  | `float32` |                                                                  |            | Standard deviation of horizontal position error, (metres) |
| epv                                                                                  | `float32` |                                                                  |            | Standard deviation of vertical position error, (metres)   |
| terrain_alt                                                     | `float32` |                                                                  |            | Terrain altitude WGS84, (metres)                          |
| terrain_alt_valid                          | `bool`    |                                                                  |            | Terrain altitude estimate is valid                                           |
| dead_reckoning                                                  | `bool`    |                                                                  |            | True if this position is estimated through dead-reckoning                    |

## Constants

| Назва                                                                | Тип      | Значення | Опис |
| -------------------------------------------------------------------- | -------- | -------- | ---- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleGlobalPosition.msg)

:::details
Click here to see original file

```c
# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#

uint32 MESSAGE_VERSION = 0

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt			# Altitude AMSL, (meters)
float32 alt_ellipsoid		# Altitude above ellipsoid, (meters)

bool lat_lon_valid
bool alt_valid

float32 delta_alt 	# Reset delta for altitude
float32 delta_terrain   # Reset delta for terrain
uint8 lat_lon_reset_counter	# Counter for reset events on horizontal position coordinates
uint8 alt_reset_counter 	# Counter for reset events on altitude
uint8 terrain_reset_counter     # Counter for reset events on terrain

float32 eph			# Standard deviation of horizontal position error, (metres)
float32 epv			# Standard deviation of vertical position error, (metres)

float32 terrain_alt		# Terrain altitude WGS84, (metres)
bool terrain_alt_valid		# Terrain altitude estimate is valid

bool dead_reckoning		# True if this position is estimated through dead-reckoning

# TOPICS vehicle_global_position vehicle_global_position_groundtruth external_ins_global_position
# TOPICS estimator_global_position
# TOPICS aux_global_position
```

:::
