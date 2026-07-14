# On-Screen Display (OSD)

An **On-Screen Display (OSD)** overlays flight telemetry — battery, altitude, GPS, RSSI, attitude, etc. — onto a pilot's video feed.
OSDs are commonly used in FPV and long-range flying so the pilot can see live flight data without looking away from the video.

PX4 supports three distinct OSD mechanisms, each targeting a different class of video system:

| Mechanism                               | Use case                                                                                                                                                                                  | Transport            | Runs on FC?                                                    |
| --------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------- | -------------------------------------------------------------- |
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

### 하드웨어 설정

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
This was common on older F4-class FCs such as OmnibusF4SD and is largely superseded by digital systems.

No external wiring is required on boards that include the chip; to enable it, set [`OSD_ATXXXX_CFG`](../advanced_config/parameter_reference.md#OSD_ATXXXX_CFG) to `1` (NTSC) or `2` (PAL) and reboot.

## See also

- [Parameter Reference > OSD](../advanced_config/parameter_reference.md#osd) — all OSD parameters.
- [MAVLink Peripherals (GCS/OSD/Gimbal/Camera/Companion)](./mavlink_peripherals.md) — MAVLink serial configuration.
- [Serial Port Configuration](./serial_configuration.md) — assigning modules to UARTs.
- [`msp_osd` module reference](../modules/modules_driver.md#msp-osd) — CLI usage and source.
