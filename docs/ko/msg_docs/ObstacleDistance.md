---
pageClass: is-wide-page
---

# ObstacleDistance (UORB message)

Obstacle distances in front of the sensor.

**TOPICS:** obstacle_distance obstacle_distance_fused

## Fields

| 명칭                                | 형식           | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| --------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                         | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                                                                                                                                                                                                             |
| frame                             | `uint8`      |                                                                  |            | Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL, which is North aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned.                                                     |
| sensor_type  | `uint8`      |                                                                  |            | Type from MAV_DISTANCE_SENSOR enum.                                                                                                                                                                                                                                                                                                                                         |
| distances                         | `uint16[72]` |                                                                  |            | Distance of obstacles around the UAV with index 0 corresponding to local North. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm. |
| increment                         | `float32`    |                                                                  |            | Angular width in degrees of each array element.                                                                                                                                                                                                                                                                                                                                                                       |
| min_distance | `uint16`     |                                                                  |            | Minimum distance the sensor can measure in centimeters.                                                                                                                                                                                                                                                                                                                                                               |
| max_distance | `uint16`     |                                                                  |            | Maximum distance the sensor can measure in centimeters.                                                                                                                                                                                                                                                                                                                                                               |
| angle_offset | `float32`    |                                                                  |            | Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward. Positive is clockwise direction, negative is counter-clockwise.                                                                                                                                                                                                               |

## Constants

| 명칭                                                                                                                                           | 형식      | Value | 설명 |
| -------------------------------------------------------------------------------------------------------------------------------------------- | ------- | ----- | -- |
| <a href="#MAV_FRAME_GLOBAL"></a> MAV_FRAME_GLOBAL                                                  | `uint8` | 0     |    |
| <a href="#MAV_FRAME_LOCAL_NED"></a> MAV_FRAME_LOCAL_NED                       | `uint8` | 1     |    |
| <a href="#MAV_FRAME_BODY_FRD"></a> MAV_FRAME_BODY_FRD                         | `uint8` | 12    |    |
| <a href="#MAV_DISTANCE_SENSOR_LASER"></a> MAV_DISTANCE_SENSOR_LASER           | `uint8` | 0     |    |
| <a href="#MAV_DISTANCE_SENSOR_ULTRASOUND"></a> MAV_DISTANCE_SENSOR_ULTRASOUND | `uint8` | 1     |    |
| <a href="#MAV_DISTANCE_SENSOR_INFRARED"></a> MAV_DISTANCE_SENSOR_INFRARED     | `uint8` | 2     |    |
| <a href="#MAV_DISTANCE_SENSOR_RADAR"></a> MAV_DISTANCE_SENSOR_RADAR           | `uint8` | 3     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ObstacleDistance.msg)

:::details
Click here to see original file

```c
# Obstacle distances in front of the sensor.
uint64 timestamp		# time since system start (microseconds)

uint8 frame		#Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL, which is North aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned.
uint8 MAV_FRAME_GLOBAL = 0
uint8 MAV_FRAME_LOCAL_NED = 1
uint8 MAV_FRAME_BODY_FRD = 12

uint8 sensor_type # Type from MAV_DISTANCE_SENSOR enum.
uint8 MAV_DISTANCE_SENSOR_LASER = 0
uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1
uint8 MAV_DISTANCE_SENSOR_INFRARED = 2
uint8 MAV_DISTANCE_SENSOR_RADAR = 3

uint16[72] distances # Distance of obstacles around the UAV with index 0 corresponding to local North. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm.

float32 increment # Angular width in degrees of each array element.

uint16 min_distance # Minimum distance the sensor can measure in centimeters.
uint16 max_distance # Maximum distance the sensor can measure in centimeters.

float32 angle_offset # Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward. Positive is clockwise direction, negative is counter-clockwise.

# TOPICS obstacle_distance obstacle_distance_fused
```

:::
