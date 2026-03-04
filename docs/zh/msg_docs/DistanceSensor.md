---
pageClass: is-wide-page
---

# DistanceSensor (UORB message)

DISTANCE_SENSOR message data.

**TOPICS:** distance_sensor

## Fields

| 参数名                                   | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                                                        |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                                                                 |
| device_id        | `uint32`     |                                                                  |            | unique device ID for the sensor that does not change between power cycles                                                                                                                                 |
| min_distance     | `float32`    |                                                                  |            | Minimum distance the sensor can measure (in m)                                                                                                                                         |
| max_distance     | `float32`    |                                                                  |            | Maximum distance the sensor can measure (in m)                                                                                                                                         |
| current_distance | `float32`    |                                                                  |            | Current distance reading (in m)                                                                                                                                                        |
| variance                              | `float32`    |                                                                  |            | Measurement variance (in m^2), 0 for unknown / invalid readings                                                                                                                        |
| signal_quality   | `int8`       |                                                                  |            | Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality. |
| type                                  | `uint8`      |                                                                  |            | Type from MAV_DISTANCE_SENSOR enum                                                                                                                              |
| h_fov            | `float32`    |                                                                  |            | Sensor horizontal field of view (rad)                                                                                                                                                  |
| v_fov            | `float32`    |                                                                  |            | Sensor vertical field of view (rad)                                                                                                                                                    |
| q                                     | `float32[4]` |                                                                  |            | Quaterion sensor orientation with respect to the vehicle body frame to specify the orientation ROTATION_CUSTOM                                                                       |
| orientation                           | `uint8`      |                                                                  |            | Direction the sensor faces from MAV_SENSOR_ORIENTATION enum                                                                                                     |
| mode                                  | `uint8`      |                                                                  |            |                                                                                                                                                                                                           |

## Constants

| 参数名                                                                                                                                          | 类型      | 值   | 描述                                                                                                                |
| -------------------------------------------------------------------------------------------------------------------------------------------- | ------- | --- | ----------------------------------------------------------------------------------------------------------------- |
| <a href="#MAV_DISTANCE_SENSOR_LASER"></a> MAV_DISTANCE_SENSOR_LASER           | `uint8` | 0   |                                                                                                                   |
| <a href="#MAV_DISTANCE_SENSOR_ULTRASOUND"></a> MAV_DISTANCE_SENSOR_ULTRASOUND | `uint8` | 1   |                                                                                                                   |
| <a href="#MAV_DISTANCE_SENSOR_INFRARED"></a> MAV_DISTANCE_SENSOR_INFRARED     | `uint8` | 2   |                                                                                                                   |
| <a href="#MAV_DISTANCE_SENSOR_RADAR"></a> MAV_DISTANCE_SENSOR_RADAR           | `uint8` | 3   |                                                                                                                   |
| <a href="#ROTATION_YAW_0		"></a> ROTATION_YAW_0                                                    | `uint8` | 0   | MAV_SENSOR_ROTATION_NONE                           |
| <a href="#ROTATION_YAW_45		"></a> ROTATION_YAW_45                                                  | `uint8` | 1   | MAV_SENSOR_ROTATION_YAW_45    |
| <a href="#ROTATION_YAW_90		"></a> ROTATION_YAW_90                                                  | `uint8` | 2   | MAV_SENSOR_ROTATION_YAW_90    |
| <a href="#ROTATION_YAW_135		"></a> ROTATION_YAW_135                                                | `uint8` | 3   | MAV_SENSOR_ROTATION_YAW_135   |
| <a href="#ROTATION_YAW_180		"></a> ROTATION_YAW_180                                                | `uint8` | 4   | MAV_SENSOR_ROTATION_YAW_180   |
| <a href="#ROTATION_YAW_225		"></a> ROTATION_YAW_225                                                | `uint8` | 5   | MAV_SENSOR_ROTATION_YAW_225   |
| <a href="#ROTATION_YAW_270		"></a> ROTATION_YAW_270                                                | `uint8` | 6   | MAV_SENSOR_ROTATION_YAW_270   |
| <a href="#ROTATION_YAW_315		"></a> ROTATION_YAW_315                                                | `uint8` | 7   | MAV_SENSOR_ROTATION_YAW_315   |
| <a href="#ROTATION_FORWARD_FACING	"></a> ROTATION_FORWARD_FACING                                   | `uint8` | 0   | MAV_SENSOR_ROTATION_NONE                           |
| <a href="#ROTATION_RIGHT_FACING	"></a> ROTATION_RIGHT_FACING                                       | `uint8` | 2   | MAV_SENSOR_ROTATION_YAW_90    |
| <a href="#ROTATION_BACKWARD_FACING	"></a> ROTATION_BACKWARD_FACING                                 | `uint8` | 4   | MAV_SENSOR_ROTATION_YAW_180   |
| <a href="#ROTATION_LEFT_FACING	"></a> ROTATION_LEFT_FACING                                         | `uint8` | 6   | MAV_SENSOR_ROTATION_YAW_270   |
| <a href="#ROTATION_UPWARD_FACING"></a> ROTATION_UPWARD_FACING                                      | `uint8` | 24  | MAV_SENSOR_ROTATION_PITCH_90  |
| <a href="#ROTATION_DOWNWARD_FACING"></a> ROTATION_DOWNWARD_FACING                                  | `uint8` | 25  | MAV_SENSOR_ROTATION_PITCH_270 |
| <a href="#ROTATION_CUSTOM"></a> ROTATION_CUSTOM                                                                         | `uint8` | 100 | MAV_SENSOR_ROTATION_CUSTOM                         |
| <a href="#MODE_UNKNOWN"></a> MODE_UNKNOWN                                                                               | `uint8` | 0   |                                                                                                                   |
| <a href="#MODE_ENABLED"></a> MODE_ENABLED                                                                               | `uint8` | 1   |                                                                                                                   |
| <a href="#MODE_DISABLED"></a> MODE_DISABLED                                                                             | `uint8` | 2   |                                                                                                                   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DistanceSensor.msg)

:::details
Click here to see original file

```c
# DISTANCE_SENSOR message data

uint64 timestamp		# time since system start (microseconds)

uint32 device_id		# unique device ID for the sensor that does not change between power cycles

float32 min_distance		# Minimum distance the sensor can measure (in m)
float32 max_distance		# Maximum distance the sensor can measure (in m)
float32 current_distance	# Current distance reading (in m)
float32 variance		# Measurement variance (in m^2), 0 for unknown / invalid readings
int8 signal_quality		# Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.

uint8 type			# Type from MAV_DISTANCE_SENSOR enum
uint8 MAV_DISTANCE_SENSOR_LASER = 0
uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1
uint8 MAV_DISTANCE_SENSOR_INFRARED = 2
uint8 MAV_DISTANCE_SENSOR_RADAR = 3

float32 h_fov # Sensor horizontal field of view (rad)
float32 v_fov # Sensor vertical field of view (rad)
float32[4] q # Quaterion sensor orientation with respect to the vehicle body frame to specify the orientation ROTATION_CUSTOM

uint8 orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum

uint8 ROTATION_YAW_0		= 0 # MAV_SENSOR_ROTATION_NONE
uint8 ROTATION_YAW_45		= 1 # MAV_SENSOR_ROTATION_YAW_45
uint8 ROTATION_YAW_90		= 2 # MAV_SENSOR_ROTATION_YAW_90
uint8 ROTATION_YAW_135		= 3 # MAV_SENSOR_ROTATION_YAW_135
uint8 ROTATION_YAW_180		= 4 # MAV_SENSOR_ROTATION_YAW_180
uint8 ROTATION_YAW_225		= 5 # MAV_SENSOR_ROTATION_YAW_225
uint8 ROTATION_YAW_270		= 6 # MAV_SENSOR_ROTATION_YAW_270
uint8 ROTATION_YAW_315		= 7 # MAV_SENSOR_ROTATION_YAW_315

uint8 ROTATION_FORWARD_FACING	= 0 # MAV_SENSOR_ROTATION_NONE
uint8 ROTATION_RIGHT_FACING	= 2 # MAV_SENSOR_ROTATION_YAW_90
uint8 ROTATION_BACKWARD_FACING	= 4 # MAV_SENSOR_ROTATION_YAW_180
uint8 ROTATION_LEFT_FACING	= 6 # MAV_SENSOR_ROTATION_YAW_270

uint8 ROTATION_UPWARD_FACING   = 24 # MAV_SENSOR_ROTATION_PITCH_90
uint8 ROTATION_DOWNWARD_FACING = 25 # MAV_SENSOR_ROTATION_PITCH_270

uint8 ROTATION_CUSTOM          = 100 # MAV_SENSOR_ROTATION_CUSTOM

uint8 mode
uint8 MODE_UNKNOWN  = 0
uint8 MODE_ENABLED  = 1
uint8 MODE_DISABLED = 2
```

:::
