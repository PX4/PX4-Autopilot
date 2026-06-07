---
pageClass: is-wide-page
---

# OpenDroneIdSystem (UORB message)

**TOPICS:** open_drone_id_system

## Fields

| Name                                                          | Type        | Unit [Frame] | Range/Enum | Description |
| ------------------------------------------------------------- | ----------- | ------------ | ---------- | ----------- |
| <a id="fld_timestamp"></a>timestamp                           | `uint64`    |              |            |
| <a id="fld_id_or_mac"></a>id_or_mac                           | `uint8[20]` |              |            |
| <a id="fld_operator_location_type"></a>operator_location_type | `uint8`     |              |            |
| <a id="fld_classification_type"></a>classification_type       | `uint8`     |              |            |
| <a id="fld_operator_latitude"></a>operator_latitude           | `int32`     |              |            |
| <a id="fld_operator_longitude"></a>operator_longitude         | `int32`     |              |            |
| <a id="fld_area_count"></a>area_count                         | `uint16`    |              |            |
| <a id="fld_area_radius"></a>area_radius                       | `uint16`    |              |            |
| <a id="fld_area_ceiling"></a>area_ceiling                     | `float32`   |              |            |
| <a id="fld_area_floor"></a>area_floor                         | `float32`   |              |            |
| <a id="fld_category_eu"></a>category_eu                       | `uint8`     |              |            |
| <a id="fld_class_eu"></a>class_eu                             | `uint8`     |              |            |
| <a id="fld_operator_altitude_geo"></a>operator_altitude_geo   | `float32`   |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OpenDroneIdSystem.msg)

::: details Click here to see original file

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
