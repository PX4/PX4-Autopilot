---
pageClass: is-wide-page
---

# SatelliteInfo (UORB message)

**TOPICS:** satellite_info

## Fields

| Name                                | Type        | Unit [Frame] | Range/Enum | Description                                                                                        |
| ----------------------------------- | ----------- | ------------ | ---------- | -------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`    |              |            | time since system start (microseconds)                                                             |
| <a id="fld_count"></a>count         | `uint8`     |              |            | Number of satellites visible to the receiver                                                       |
| <a id="fld_svid"></a>svid           | `uint8[40]` |              |            | Space vehicle ID [1..255], see scheme below                                                        |
| <a id="fld_used"></a>used           | `uint8[40]` |              |            | 0: Satellite not used, 1: used for navigation                                                      |
| <a id="fld_elevation"></a>elevation | `uint8[40]` |              |            | Elevation (0: right on top of receiver, 90: on the horizon) of satellite                           |
| <a id="fld_azimuth"></a>azimuth     | `uint8[40]` |              |            | Direction of satellite, 0: 0 deg, 255: 360 deg.                                                    |
| <a id="fld_snr"></a>snr             | `uint8[40]` |              |            | dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite. |
| <a id="fld_prn"></a>prn             | `uint8[40]` |              |            | Satellite PRN code assignment, (psuedorandom number SBAS, valid codes are 120-144)                 |

## Constants

| Name                                                          | Type    | Value | Description |
| ------------------------------------------------------------- | ------- | ----- | ----------- |
| <a id="#SAT_INFO_MAX_SATELLITES"></a> SAT_INFO_MAX_SATELLITES | `uint8` | 40    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SatelliteInfo.msg)

::: details Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
uint8 SAT_INFO_MAX_SATELLITES = 40

uint8 count			# Number of satellites visible to the receiver
uint8[40] svid	 		# Space vehicle ID [1..255], see scheme below
uint8[40] used			# 0: Satellite not used, 1: used for navigation
uint8[40] elevation		# Elevation (0: right on top of receiver, 90: on the horizon) of satellite
uint8[40] azimuth		# Direction of satellite, 0: 0 deg, 255: 360 deg.
uint8[40] snr			# dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite.
uint8[40] prn                   # Satellite PRN code assignment, (psuedorandom number SBAS, valid codes are 120-144)
```

:::
