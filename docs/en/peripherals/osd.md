# On-Screen Display (OSD)

An **On-Screen Display (OSD)** overlays flight telemetry — battery, altitude, GPS, RSSI, attitude, etc. — onto a pilot's video feed.
OSDs are commonly used in FPV and long-range flying so the pilot can see live flight data without looking away from the video.

PX4 supports three distinct OSD mechanisms, each targeting a different class of video system:

| Mechanism                               | Use case                                                                                                                               | Transport            | Runs on FC?                                                    |
| --------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- | -------------------- | -------------------------------------------------------------- |
| [MSP OSD](#msp-osd)                     | Digital FPV air units and video goggles that speak Betaflight MSP (e.g. DJI O3/O4, Walksnail, HDZero, Caddx Vista)                     | Serial, MSPv1        | Yes — [`msp_osd`](../modules/modules_driver.md#msp-osd) driver |
| [ATXXXX Analog OSD](#atxxxx-analog-osd) | Legacy analog video with an on-board MAX7456/ATXXXX overlay chip (e.g. OmnibusF4SD)                                                    | SPI to on-board chip | Yes — [`atxxxx`](../modules/modules_driver.md#atxxxx) driver   |
| [MAVLink OSD](#mavlink-osd)             | MAVLink-aware ground stations and displays that render their own OSD from telemetry (e.g. Yaapu on EdgeTX/OpenTX, Skydroid, mLRS HUDs) | Serial, MAVLink      | No — streams MAVLink; the display renders the OSD              |

Which one you use is determined by your video hardware, not by PX4 preference.
If you're unsure, start with your video system's documentation and match the OSD mechanism it expects.

## MSP OSD

**MSP (MultiWii Serial Protocol) OSD** is the mechanism used by digital FPV systems (DJI, Walksnail, HDZero) and by many digital goggles/air units to render telemetry over the pilot's video feed.
PX4 implements the subset of MSP used for OSD telemetry, matching what Betaflight and INAV send.

The [`msp_osd`](../modules/modules_driver.md#msp-osd) driver converts uORB topics (battery, GPS, attitude, etc.) to MSP packets and sends them out a serial port at 115200 baud.

### Supported displays

PX4 currently sends a subset of MSP messages.
Reliably-working display items include:

- Craft name and flight mode / arming state
- Battery voltage, current draw, mAh consumed, average cell voltage
- GPS latitude, longitude, satellite count, ground speed
- Home distance and direction
- Altitude (from GNSS / baro)
- RSSI
- Crosshairs toggle

Some items in [`OSD_SYMBOLS`](../advanced_config/parameter_reference.md#OSD_SYMBOLS) are reserved but not yet implemented — see the parameter's `(unused)` bit labels.
For feature-completeness work, see the tracking issues on GitHub.

### Hardware setup

1. Connect the digital air unit's MSP / telemetry input to a free UART on the flight controller (TX → RX, RX → TX, GND → GND).
2. Power the air unit from its own BEC or a VTX power pad — most air units expect 5 V or battery voltage, not autopilot 5 V.
3. Note which PX4 serial device the UART maps to on your board (e.g. `TELEM2` → `/dev/ttyS2`).
   See [Serial Port Mapping](../hardware/serial_port_mapping.md).

### Firmware requirements

The `msp_osd` driver is included in the default build for most modern Pixhawk and FPV-oriented boards (e.g. `px4_fmu-v5x`, `px4_fmu-v6x`, `ark_fpv`, `cuav_7-nano`, `micoair_h743*`).
If your board does not include it by default, enable it via [board config](../hardware/porting_guide_config.md#px4-menuconfig-setup):

```sh
make <board>_default boardconfig
# drivers → OSD → msp_osd
```

Then rebuild and flash.

### PX4 configuration

1. Assign the selected serial port to MSP OSD with [`MSP_OSD_CONFIG`](../advanced_config/parameter_reference.md#MSP_OSD_CONFIG).
2. Set the matching `SER_<PORT>_BAUD` to `115200`.
3. Reboot.
4. Tune the display via the [`OSD_*` parameters](../advanced_config/parameter_reference.md#osd):
   - [`OSD_SYMBOLS`](../advanced_config/parameter_reference.md#OSD_SYMBOLS) — bitmask selecting which items appear.
   - [`OSD_CH_HEIGHT`](../advanced_config/parameter_reference.md#OSD_CH_HEIGHT) — vertical position of the crosshairs.
   - [`OSD_LOG_LEVEL`](../advanced_config/parameter_reference.md#OSD_LOG_LEVEL) — minimum severity for on-screen warnings.
   - [`OSD_SCROLL_RATE`](../advanced_config/parameter_reference.md#OSD_SCROLL_RATE) / [`OSD_DWELL_TIME`](../advanced_config/parameter_reference.md#OSD_DWELL_TIME) — scrolling of long messages.
   - [`OSD_RC_STICK`](../advanced_config/parameter_reference.md#OSD_RC_STICK) — forward RC sticks to the VTX when disarmed, so you can navigate the VTX menu.

### Worked examples

- [Reptile Dragon 2 > msp_osd Module](../frames_plane/reptile_dragon_2.md#msp-osd-module) — end-to-end wiring and configuration for a Caddx Vista build.
- [Turbo Timber Evolution](../frames_plane/turbo_timber_evolution.md) — references the same setup pattern.

## MAVLink OSD

Some OSDs render their own overlay directly from the MAVLink telemetry stream — the flight controller simply streams MAVLink at a rate the display can parse.
PX4 exposes this via a dedicated MAVLink stream profile.

To use a MAVLink OSD:

1. Choose an unused MAVLink instance ([`MAV_X_CONFIG`](../peripherals/mavlink_peripherals.md#default_ports)) and assign it to the serial port connected to the display.
2. Configure the mode of the selected MAVLink instance with [`MAV_X_MODE`](./mavlink_peripherals.md#MAV_X_MODE) by setting it to **`OSD`**.
   The `OSD` mode uses a built-in rate table tuned for low-bandwidth OSD consumption.
3. Set the matching `SER_<PORT>_BAUD` to the baud rate the display expects.

The stream content is fixed (defined in `src/modules/mavlink/mavlink_main.cpp`) and cannot be customised from parameters.
See [MAVLink Peripherals (GCS/OSD/Gimbal/Camera/Companion)](./mavlink_peripherals.md) for the full MAVLink-side configuration.

## ATXXXX Analog OSD

The [`atxxxx`](../modules/modules_driver.md#atxxxx) driver targets boards with an on-board MAX7456 / ATXXXX chip that overlays characters onto an analog video stream (PAL or NTSC).

The OSD chip is connected to PX4 over SPI and inserted between the camera input and analog video output by the flight controller hardware.
Use the board documentation to identify its camera input, video output, and video ground pads.

### Firmware Support

The board firmware must include the `atxxxx` driver, and the board must define the SPI bus and chip-select wiring for its OSD chip.
Supported boards normally enable the driver with `CONFIG_DRIVERS_OSD_ATXXXX=y` and start it from the board startup script when [`OSD_ATXXXX_CFG`](../advanced_config/parameter_reference.md#OSD_ATXXXX_CFG) is non-zero.

Enabling the driver in the board configuration is not sufficient when porting an unsupported board: the board SPI configuration and startup script must also describe and start the physical device.
See [PX4 Board Configuration](../hardware/porting_guide_config.md) when adding support for another board.

### Setup

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

### Display Elements

[`OSD_SYMBOLS`](../advanced_config/parameter_reference.md#OSD_SYMBOLS) is a bitmask that selects the displayed elements.
Its default value, `16383`, enables bits 0 through 13.
The ATXXXX meanings and corresponding position parameters are:

| Bit | Element                                                     | Position parameters                  |
| --: | ----------------------------------------------------------- | ------------------------------------ |
|   0 | MAVLink system ID                                           | `OSD_SYSID_X`, `OSD_SYSID_Y`         |
|   1 | Armed/disarmed state                                        | `OSD_ARM_X`, `OSD_ARM_Y`             |
|   2 | GPS latitude                                                | `OSD_GPS_LAT_X`, `OSD_GPS_LAT_Y`     |
|   3 | GPS longitude                                               | `OSD_GPS_LON_X`, `OSD_GPS_LON_Y`     |
|   4 | GPS satellite count                                         | `OSD_GPS_SAT_X`, `OSD_GPS_SAT_Y`     |
|   5 | GPS ground speed in km/h                                    | `OSD_GPS_SPD_X`, `OSD_GPS_SPD_Y`     |
|   6 | Distance to home in metres                                  | `OSD_HOME_DST_X`, `OSD_HOME_DST_Y`   |
|   7 | Mission state or sequence progress                          | `OSD_MISSION_X`, `OSD_MISSION_Y`     |
|   8 | Main battery voltage                                        | `OSD_BAT_VOLT_X`, `OSD_BAT_VOLT_Y`   |
|   9 | Battery current in A                                        | `OSD_CURRENT_X`, `OSD_CURRENT_Y`     |
|  10 | Consumed battery capacity in mAh                            | `OSD_MAH_X`, `OSD_MAH_Y`             |
|  11 | RC RSSI                                                     | `OSD_RSSI_X`, `OSD_RSSI_Y`           |
|  12 | Local altitude in metres                                    | `OSD_ALT_X`, `OSD_ALT_Y`             |
|  13 | Vertical speed in m/s                                       | `OSD_VARIO_X`, `OSD_VARIO_Y`         |
|  14 | Flight mode                                                 | `OSD_MODE_X`, `OSD_MODE_Y`           |
|  15 | RC link quality                                             | `OSD_LQ_X`, `OSD_LQ_Y`               |
|  16 | Pitch in degrees                                            | `OSD_PITCH_X`, `OSD_PITCH_Y`         |
|  17 | Roll in degrees                                             | `OSD_ROLL_X`, `OSD_ROLL_Y`           |
|  18 | Center crosshair                                            | `OSD_CROSS_X`, `OSD_CROSS_Y`         |
|  19 | Average battery cell voltage                                | `OSD_CELL_V_X`, `OSD_CELL_V_Y`       |
|  20 | MAV state                                                   | `OSD_MAV_STATE_X`, `OSD_MAV_STATE_Y` |
|  21 | Electrical power in W                                       | `OSD_POWER_X`, `OSD_POWER_Y`         |
|  22 | Elapsed armed flight time                                   | `OSD_FTIME_X`, `OSD_FTIME_Y`         |
|  23 | PX4 log messages and vehicle status                         | `OSD_STATUS_X`, `OSD_STATUS_Y`       |
|  24 | Artificial horizon                                          | `OSD_AH_X`, `OSD_AH_Y`               |
|  25 | Heading in degrees                                          | `OSD_HEAD_X`, `OSD_HEAD_Y`           |
|  26 | VTX band, channel, and power level                          | `OSD_VTX_INFO_X`, `OSD_VTX_INFO_Y`   |
|  27 | VTX frequency in MHz                                        | `OSD_VTX_FREQ_X`, `OSD_VTX_FREQ_Y`   |
|  28 | VTX power label                                             | `OSD_VTX_POWER_X`, `OSD_VTX_POWER_Y` |
|  29 | Throttle percentage                                         | `OSD_THROT_X`, `OSD_THROT_Y`         |
|  30 | GPS fix type, PDOP, and estimated horizontal position error | `OSD_GPS_INFO_X`, `OSD_GPS_INFO_Y`   |

Set and clear bits in QGroundControl's bitmask parameter editor, or enter the combined decimal value directly.

### Positioning

Each element has separate `_X` and `_Y` parameters.
The origin is the top-left character cell: X increases to the right and Y increases downward.
The permitted X range depends on the width of the element so that its complete text fits within the 30-column display.
The configurable Y range is 0 through 12.

Position changes are applied while the driver is running.
When elements overlap, the element rendered later by the driver occupies the shared character cells.

### Status Messages

The status element at `OSD_STATUS_X` and `OSD_STATUS_Y` displays PX4 log messages that pass the [`OSD_LOG_LEVEL`](../advanced_config/parameter_reference.md#OSD_LOG_LEVEL) filter.
Messages wider than the available field scroll according to [`OSD_SCROLL_RATE`](../advanced_config/parameter_reference.md#OSD_SCROLL_RATE) and pause at the beginning according to [`OSD_DWELL_TIME`](../advanced_config/parameter_reference.md#OSD_DWELL_TIME).

When there is no log message to show, the field reports relevant vehicle state such as readiness, arming, failsafe, failure detector events, battery warning, RC loss, or invalid attitude, position, GPS, and home data.

### Artificial Horizon

Enable bit 24 in `OSD_SYMBOLS` and position the horizon center with `OSD_AH_X` and `OSD_AH_Y`.
The projection compensates for the camera geometry using:

- `OSD_CAM_HFOV`: horizontal camera field of view in degrees
- `OSD_CAM_VFOV`: vertical camera field of view in degrees
- `OSD_CAM_UPT`: upward camera mounting angle in degrees

Set the FOV parameters from the camera specification and set `OSD_CAM_UPT` to the physical uptilt of the FPV camera.
The separately selectable crosshair uses `OSD_CROSS_X` and `OSD_CROSS_Y`.

### VTX Information

The VTX elements require firmware built with `CONFIG_DRIVERS_VTX` and a configured [analog VTX driver](../vtx/index.md).
The OSD reads the current band, channel, frequency, and power information reported by the VTX driver.
Enable bits 26, 27, and 28 in `OSD_SYMBOLS` to display those fields.

### Troubleshooting

If no overlay is visible:

1. Confirm that `OSD_ATXXXX_CFG` is non-zero and matches the camera and receiver video standard.
2. Reboot and check `atxxxx status`.
3. Confirm that the board firmware includes the driver and that its startup script starts the correct SPI device.
4. Check the board-specific camera input, video output, and ground wiring.

If an element is missing, confirm that its `OSD_SYMBOLS` bit is enabled and that its position does not overlap another enabled element.
If the artificial horizon is offset or moves by the wrong scale, verify the camera FOV and uptilt parameters.

## See also

- [Parameter Reference > OSD](../advanced_config/parameter_reference.md#osd) — all OSD parameters.
- [MAVLink Peripherals (GCS/OSD/Gimbal/Camera/Companion)](./mavlink_peripherals.md) — MAVLink serial configuration.
- [Serial Port Configuration](./serial_configuration.md) — assigning modules to UARTs.
- [`msp_osd` module reference](../modules/modules_driver.md#msp-osd) — CLI usage and source.
