---
pageClass: is-wide-page
---

# OpenDroneIdSystem (UORB message)

**TOPICS:** open_droneid_system

## Fields

| 参数名                                                              | 类型          | Unit [Frame] | Range/Enum | 描述 |
| ---------------------------------------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | -- |
| timestamp                                                        | `uint64`    |                                                                  |            |    |
| id_or_mac              | `uint8[20]` |                                                                  |            |    |
| operator_location_type | `uint8`     |                                                                  |            |    |
| classification_type                         | `uint8`     |                                                                  |            |    |
| operator_latitude                           | `int32`     |                                                                  |            |    |
| operator_longitude                          | `int32`     |                                                                  |            |    |
| area_count                                  | `uint16`    |                                                                  |            |    |
| area_radius                                 | `uint16`    |                                                                  |            |    |
| area_ceiling                                | `float32`   |                                                                  |            |    |
| area_floor                                  | `float32`   |                                                                  |            |    |
| category_eu                                 | `uint8`     |                                                                  |            |    |
| class_eu                                    | `uint8`     |                                                                  |            |    |
| operator_altitude_geo  | `float32`   |                                                                  |            |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OpenDroneIdSystem.msg)

:::details
Click here to see original file

```c
uint64 timestamp
uint8[20] id_or_mac
uint8 operator_location_type
uint8 classification_type
int32 operator_latitude
int32 operator_longitude
uint16 area_count
uint16 area_radius
float32 area_ceiling
float32 area_floor
uint8 category_eu
uint8 class_eu
float32 operator_altitude_geo
```

:::
