# ATXXXX Analog OSD

The [`atxxxx`](../modules/modules_driver.md#atxxxx) driver targets boards with an on-board MAX7456 / ATXXXX chip that overlays characters onto an analog video stream (PAL or NTSC).

The OSD chip is connected to PX4 over SPI and inserted between the camera input and analog video output by the flight controller hardware.
Use the board documentation to identify its camera input, video output, and video ground pads.

## Firmware Support

The board must have an on-board OSD chip and firmware that includes the `atxxxx` driver.
On supported boards the driver starts automatically when [`OSD_ATXXXX_CFG`](../advanced_config/parameter_reference.md#OSD_ATXXXX_CFG) is non-zero.

## Setup

1. Connect the analog camera and video transmitter or display to the flight controller's video input and output according to the board documentation.
2. Flash firmware that includes the `atxxxx` driver.
3. Set [`OSD_ATXXXX_CFG`](../advanced_config/parameter_reference.md#OSD_ATXXXX_CFG) to match the video standard:
   - `0`: disabled
   - `1`: NTSC
   - `2`: PAL
4. Reboot the flight controller.
5. Verify that the driver is running from the MAVLink console:

   ```sh
   atxxxx status
   ```

The camera, OSD, and video receiver must use a compatible video standard.

## Display Elements

[`OSD_SYMBOLS`](../advanced_config/parameter_reference.md#OSD_SYMBOLS) is a bitmask that selects the displayed elements.
Its default value, `16383`, enables bits 0 through 13, so the analog-only elements on bits 22 and up are off until enabled.
Bits absent from the table below are defined by the MSP OSD and are not drawn by this driver.
The ATXXXX meanings and corresponding position parameters are:

| Bit | Element                                                     | Position parameters                  |
| --: | ----------------------------------------------------------- | ------------------------------------ |
|   0 | MAVLink system ID and MAV state, per `OSD_ID_MODE`          | `OSD_SYSID_X`, `OSD_SYSID_Y`         |
|   1 | Armed/disarmed state                                        | `OSD_ARM_X`, `OSD_ARM_Y`             |
|   2 | GPS latitude                                                | `OSD_GPS_LAT_X`, `OSD_GPS_LAT_Y`     |
|   3 | GPS longitude                                               | `OSD_GPS_LON_X`, `OSD_GPS_LON_Y`     |
|   4 | GPS satellite count                                         | `OSD_GPS_SAT_X`, `OSD_GPS_SAT_Y`     |
|   5 | GPS ground speed in km/h                                    | `OSD_GPS_SPD_X`, `OSD_GPS_SPD_Y`     |
|   6 | Distance to home in metres                                  | `OSD_HOME_DST_X`, `OSD_HOME_DST_Y`   |
|   8 | Battery voltage, per `OSD_VOLT_MODE`                        | `OSD_BAT_VOLT_X`, `OSD_BAT_VOLT_Y`   |
|   9 | Power draw, per `OSD_PWR_MODE`                              | `OSD_CURRENT_X`, `OSD_CURRENT_Y`     |
|  10 | Consumed battery capacity in mAh                            | `OSD_MAH_X`, `OSD_MAH_Y`             |
|  11 | RC RSSI                                                     | `OSD_RSSI_X`, `OSD_RSSI_Y`           |
|  12 | Local altitude in metres                                    | `OSD_ALT_X`, `OSD_ALT_Y`             |
|  13 | Vertical speed in m/s                                       | `OSD_VARIO_X`, `OSD_VARIO_Y`         |
|  14 | Flight mode                                                 | `OSD_MODE_X`, `OSD_MODE_Y`           |
|  16 | Pitch and roll in degrees                                   | `OSD_PITCH_X`, `OSD_PITCH_Y`         |
|  18 | Centre crosshair                                            | `OSD_CH_POS_HOR`, `OSD_CH_POS_VER`   |
|  22 | Mission state or sequence progress                          | `OSD_MISSION_X`, `OSD_MISSION_Y`     |
|  23 | RC link quality                                             | `OSD_LQ_X`, `OSD_LQ_Y`               |
|  24 | Elapsed armed flight time                                   | `OSD_FTIME_X`, `OSD_FTIME_Y`         |
|  25 | PX4 log messages and vehicle status                         | `OSD_STATUS_X`, `OSD_STATUS_Y`       |
|  26 | Artificial horizon                                          | `OSD_AH_X`, `OSD_AH_Y`               |
|  27 | Heading in degrees                                          | `OSD_HEAD_X`, `OSD_HEAD_Y`           |
|  28 | VTX status, per `OSD_VTX_MODE`                              | `OSD_VTX_INFO_X`, `OSD_VTX_INFO_Y`   |
|  29 | Throttle percentage                                         | `OSD_THROT_X`, `OSD_THROT_Y`         |
|  30 | GPS fix type, PDOP, and estimated horizontal position error | `OSD_GPS_INFO_X`, `OSD_GPS_INFO_Y`   |

Set and clear bits in QGroundControl's bitmask parameter editor, or enter the combined decimal value directly.

## Positioning

Each element has separate `_X` and `_Y` parameters.
The origin is the top-left character cell: X increases to the right and Y increases downward.
The X range is 0 through 29 and the Y range is 0 through 15, which are the PAL screen dimensions.
Anything falling outside the screen is clipped rather than wrapped, so an element placed near an edge simply loses the characters that do not fit.
NTSC has 13 rows, so on NTSC an element positioned at row 13, 14, or 15 is not drawn.

For most elements X is the leftmost character cell.
The flight mode, status message, and arming state hold text of varying length, so for those X is the centre of the text and the element grows symmetrically around it.

Position changes are applied while the driver is running.
When elements overlap, the element rendered later by the driver occupies the shared character cells.

## Mission State

The mission element at `OSD_MISSION_X` and `OSD_MISSION_Y` reports the state of the stored mission.

| Display    | Meaning                                                               |
| ---------- | --------------------------------------------------------------------- |
| `MISNONE`  | No mission is stored                                                  |
| `MISWAIT`  | A mission is stored but has not been checked yet                      |
| `MISFAIL`  | The mission cannot continue or be completed                           |
| `MISINVAL` | The mission was checked and rejected as infeasible                    |
| `MISDONE`  | The mission ran to completion                                         |
| `MISWARN`  | The mission is valid but contains items flagged as potentially unsafe |
| `MIS07/12` | Current item and total item count, each clamped to 99                 |

PX4 checks a mission only once a home position, a global position, and the geofence are all available.
Until then a freshly uploaded mission shows `MISWAIT`, which most often means the vehicle does not yet have a GPS fix.

## Status Messages

The status element at `OSD_STATUS_X` and `OSD_STATUS_Y` displays PX4 log messages that pass the [`OSD_LOG_LEVEL`](../advanced_config/parameter_reference.md#OSD_LOG_LEVEL) filter.
Messages wider than the available field scroll according to [`OSD_SCROLL_RATE`](../advanced_config/parameter_reference.md#OSD_SCROLL_RATE) and pause at the beginning according to [`OSD_DWELL_TIME`](../advanced_config/parameter_reference.md#OSD_DWELL_TIME).

When there is no log message to show, the field reports an abnormal vehicle state such as failsafe, failure detector events, battery warning, RC loss, or invalid attitude, position, GPS, and home data. It stays blank when nothing is wrong; the element on bit 1 reports armed state.

## Artificial Horizon

Enable bit 26 in `OSD_SYMBOLS` and position the horizon centre with `OSD_AH_X` and `OSD_AH_Y`.
The projection compensates for the camera geometry using:

- `OSD_CAM_HFOV`: horizontal camera field of view in degrees
- `OSD_CAM_VFOV`: vertical camera field of view in degrees
- `OSD_CAM_UPT`: upward camera mounting angle in degrees

Set the FOV parameters from the camera specification and set `OSD_CAM_UPT` to the physical uptilt of the FPV camera.
The separately selectable crosshair uses `OSD_CH_POS_HOR` and `OSD_CH_POS_VER`, which are offsets
from the centre of the screen and are shared with the MSP OSD.

## VTX Information

The VTX elements require firmware built with `CONFIG_DRIVERS_VTX` and a configured [analog VTX driver](../vtx/index.md).
The OSD reads the current band, channel, frequency, and power information reported by the VTX driver.
Enable bit 28 in `OSD_SYMBOLS` and select what the element shows with `OSD_VTX_MODE`: either the
configured band, channel and power level, or the resulting frequency and output power in mW.

## Troubleshooting

If no overlay is visible:

1. Confirm that `OSD_ATXXXX_CFG` is non-zero and matches the camera and receiver video standard.
2. Reboot and check `atxxxx status`.
3. Confirm that the firmware includes the `atxxxx` driver.
4. Check the board-specific camera input, video output, and ground wiring.

If an element is missing, confirm that its `OSD_SYMBOLS` bit is enabled and that its position does not overlap another enabled element.
If the artificial horizon is offset or moves by the wrong scale, verify the camera FOV and uptilt parameters.
