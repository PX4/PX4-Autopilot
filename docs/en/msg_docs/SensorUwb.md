---
pageClass: is-wide-page
---

# SensorUwb (UORB message)

Ultra-wideband (UWB) distance sensor.

Distance and angle-of-arrival ranging data from an ultra-wideband (UWB) positioning
system, such as Pozyx or NXP Rddrone. Published and consumed internally by the UWB
ranging driver (src/drivers/uwb/uwb_sr150), which parses serial ranging results from
the sensor module and publishes them on this topic; no other module in the codebase
currently subscribes to it.

**TOPICS:** sensor_uwb

## Fields

| Name                                                          | Type      | Unit [Frame] | Range/Enum | Description                                                                                                                                                |
| ------------------------------------------------------------- | --------- | ------------ | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                           | `uint64`  | us           |            | Time since system start                                                                                                                                    |
| <a id="fld_sessionid"></a>sessionid                           | `uint32`  |              |            | UWB SessionID                                                                                                                                              |
| <a id="fld_time_offset"></a>time_offset                       | `uint32`  |              |            | Time between Ranging Rounds in ms                                                                                                                          |
| <a id="fld_counter"></a>counter                               | `uint32`  |              |            | Number of Ranges since last Start of Ranging                                                                                                               |
| <a id="fld_mac"></a>mac                                       | `uint16`  |              |            | MAC address of Initiator (controller)                                                                                                                      |
| <a id="fld_mac_dest"></a>mac_dest                             | `uint16`  |              |            | MAC address of Responder (Controlee)                                                                                                                       |
| <a id="fld_status"></a>status                                 | `uint16`  |              |            | Ranging status code passed through from the UWB module's measurement result                                                                                |
| <a id="fld_nlos"></a>nlos                                     | `uint8`   |              |            | Non-line-of-sight condition (y/n)                                                                                                                          |
| <a id="fld_distance"></a>distance                             | `float32` | m            |            | Distance to the UWB receiver                                                                                                                               |
| <a id="fld_aoa_azimuth_dev"></a>aoa_azimuth_dev               | `float32` | deg          |            | Angle of arrival of first incoming RX msg                                                                                                                  |
| <a id="fld_aoa_elevation_dev"></a>aoa_elevation_dev           | `float32` | deg          |            | Angle of arrival of first incoming RX msg                                                                                                                  |
| <a id="fld_aoa_azimuth_resp"></a>aoa_azimuth_resp             | `float32` | deg          |            | Angle of arrival of first incoming RX msg at the responder                                                                                                 |
| <a id="fld_aoa_elevation_resp"></a>aoa_elevation_resp         | `float32` | deg          |            | Angle of arrival of first incoming RX msg at the responder                                                                                                 |
| <a id="fld_aoa_azimuth_fom"></a>aoa_azimuth_fom               | `uint8`   |              |            | AOA Azimuth Figure of Merit (FOM)                                                                                                                          |
| <a id="fld_aoa_elevation_fom"></a>aoa_elevation_fom           | `uint8`   |              |            | AOA Elevation FOM                                                                                                                                          |
| <a id="fld_aoa_dest_azimuth_fom"></a>aoa_dest_azimuth_fom     | `uint8`   |              |            | AOA Azimuth FOM                                                                                                                                            |
| <a id="fld_aoa_dest_elevation_fom"></a>aoa_dest_elevation_fom | `uint8`   |              |            | AOA Elevation FOM                                                                                                                                          |
| <a id="fld_orientation"></a>orientation                       | `uint8`   |              |            | Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. Standard configuration has antennas facing down and azimuth aligned in the forward direction. |
| <a id="fld_offset_x"></a>offset_x                             | `float32` |              |            | UWB initiator offset in X axis (NED drone frame)                                                                                                           |
| <a id="fld_offset_y"></a>offset_y                             | `float32` |              |            | UWB initiator offset in Y axis (NED drone frame)                                                                                                           |
| <a id="fld_offset_z"></a>offset_z                             | `float32` |              |            | UWB initiator offset in Z axis (NED drone frame)                                                                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorUwb.msg)

::: details Click here to see original file

```c
# Ultra-wideband (UWB) distance sensor
#
# Distance and angle-of-arrival ranging data from an ultra-wideband (UWB) positioning
# system, such as Pozyx or NXP Rddrone. Published and consumed internally by the UWB
# ranging driver (src/drivers/uwb/uwb_sr150), which parses serial ranging results from
# the sensor module and publishes them on this topic; no other module in the codebase
# currently subscribes to it.

uint64 timestamp # [us] Time since system start

uint32 sessionid # UWB SessionID
uint32 time_offset # Time between Ranging Rounds in ms
uint32 counter # Number of Ranges since last Start of Ranging
uint16 mac # [-] MAC address of Initiator (controller)

uint16 mac_dest # [-] MAC address of Responder (Controlee)
uint16 status # [-] Ranging status code passed through from the UWB module's measurement result
uint8 nlos # [-] Non-line-of-sight condition (y/n)
float32 distance # [m] Distance to the UWB receiver

#Angle of arrival, Angle in Degree -60..+60; FOV in both axis is 120 degrees
float32 aoa_azimuth_dev # [deg] Angle of arrival of first incoming RX msg
float32 aoa_elevation_dev # [deg] Angle of arrival of first incoming RX msg
float32 aoa_azimuth_resp # [deg] Angle of arrival of first incoming RX msg at the responder
float32 aoa_elevation_resp # [deg] Angle of arrival of first incoming RX msg at the responder

# Figure of merit for the angle measurements
uint8 aoa_azimuth_fom # AOA Azimuth Figure of Merit (FOM)
uint8 aoa_elevation_fom # AOA Elevation FOM
uint8 aoa_dest_azimuth_fom # AOA Azimuth FOM
uint8 aoa_dest_elevation_fom # AOA Elevation FOM

# Initiator physical configuration
uint8 orientation # Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. Standard configuration has antennas facing down and azimuth aligned in the forward direction.
float32 offset_x # UWB initiator offset in X axis (NED drone frame)
float32 offset_y # UWB initiator offset in Y axis (NED drone frame)
float32 offset_z # UWB initiator offset in Z axis (NED drone frame)
```

:::
