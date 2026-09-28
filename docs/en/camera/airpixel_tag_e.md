# AirPixel TAG-E (Sony ILX-LR1 Camera Control and Geotagging)

The [AirPixel TAG-E](https://airpixel.cz/) is a camera controller and EXIF/XMP geotagger for the Sony ILX-LR1.
It connects to the camera with a single USB-C cable and to PX4 over MAVLink, and acts as a [MAVLink camera](../camera/mavlink_v2_camera.md) using [Camera Protocol v2](https://mavlink.io/en/services/camera.html).

QGroundControl detects the camera automatically and shows its shutter, video, exposure and configuration controls.
TAG-E writes position, altitude and camera attitude into the EXIF and XMP of every photo on the camera's SD card during the flight, so no log-based geotagging is needed afterwards.

<img src="https://airpixel.cz/wp-content/uploads/2025/07/shop-tage-coin.webp" width="300" alt="AirPixel TAG-E">

## Key Features

- Full MAVLink Camera Protocol v2 control of the Sony ILX-LR1 (ISO, aperture, shutter speed, exposure compensation, exposure mode, AF mode, photo/video).
- Automatic EXIF and XMP geotagging with position, altitude and camera roll/pitch/yaw, at the camera's full trigger rate.
- Clock synchronisation with the flight controller using the MAVLink [TIMESYNC](https://mavlink.io/en/services/timesync.html) protocol.
- Lever-arm correction from antenna-to-camera offsets, optional rangefinder, GPS/IMU accuracy and custom label tags, PPK output, and video geotagging via subtitles.
- Camera clock set automatically from GPS time.

## Wiring

Connect TAG-E's IO-B connector to a TELEM port on the flight controller (e.g. `TELEM1` or `TELEM2`), and TAG-E's USB-C port to the camera.
TAG-E needs a 5V supply able to deliver at least 2A.

Pinouts are in the [TAG-E pinout manual](https://airpixel.cz/docs/tag-e-pinouts/).

## PX4 Configuration

TAG-E uses the component ID `MAV_COMP_ID_CAMERA` (100).
Configure the MAVLink instance on the port it is connected to (`MAV_1_*` or `MAV_2_*`):

| Parameter                                                   | Value                  |
| ----------------------------------------------------------- | ---------------------- |
| [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG) | `TELEM 1` or `TELEM 2` |
| [SER_TEL1_BAUD](../advanced_config/parameter_reference.md#SER_TEL1_BAUD) (or `SER_TEL2_BAUD`) | `921600` |
| [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE) | `Onboard`             |
| [MAV_1_RATE](../advanced_config/parameter_reference.md#MAV_1_RATE) | `0`                   |
| [MAV_1_FORWARD](../advanced_config/parameter_reference.md#MAV_1_FORWARD) | `Enabled`       |

Reboot the flight controller.
When the camera is online, the TAG-E LED turns green and the camera panel appears in QGroundControl.

TAG-E requests the messages it needs using `SET_MESSAGE_INTERVAL`: `TIMESYNC`, `SYSTEM_TIME` (1 Hz), `GPS_RAW_INT`, `GLOBAL_POSITION_INT` and `ATTITUDE` (10 Hz).

:::tip
TAG-E units with firmware older than `0_193` use 230400 baud.
See the manufacturer's [PX4 connection manual](https://airpixel.cz/docs/tag-e-mavlink-connection-px4/) for changing the baud rate, and the [GPS delay calculation](https://airpixel.cz/docs/gps-delay-calculation/) for the best geotag accuracy.
:::

## Camera Control in QGroundControl

When the camera is online, QGroundControl shows the camera panel on the right of the Fly view: the shutter button, a photo/video switch, and the photo count or recording time.
The gear icon under the shutter button opens the camera settings: exposure mode, aperture, shutter speed, ISO, focus mode and image resolution, plus TAG-E functions such as remote SD card format, a custom EXIF label, landing detection and photo mode.

<a href="https://airpixel.cz/wp-content/uploads/2026/09/QGC-UI-TAG-E.webp"><img src="https://airpixel.cz/wp-content/uploads/2026/09/QGC-UI-TAG-E.webp" width="100" alt="QGroundControl camera settings for the Sony ILX-LR1 with TAG-E"></a>

## Triggering

PX4 re-emits the image and video capture commands in missions (`MAV_CMD_IMAGE_START_CAPTURE`, `MAV_CMD_IMAGE_STOP_CAPTURE`, `MAV_CMD_VIDEO_START_CAPTURE`, `MAV_CMD_VIDEO_STOP_CAPTURE`) to `MAV_COMP_ID_CAMERA`, so they reach TAG-E directly.
The camera can also be triggered from the QGroundControl camera panel.

## Further Information

- [TAG-E product page](https://airpixel.cz/)
- [PX4 camera control guide for the ILX-LR1](https://airpixel.cz/nw/guides/px4-ilx-lr1-camera-control/) (AirPixel)
- [TAG-E documentation](https://airpixel.cz/docs-tag-e/)
- [MAVLink Cameras (Camera Protocol v2)](../camera/mavlink_v2_camera.md)
