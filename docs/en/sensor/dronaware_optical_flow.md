# Dronaware Optical Flow (ESP32 Camera Boards)

[Dronaware Optical Flow](https://dronaware.com/opticalflow/) is licensed firmware that can be flashed on an off-the-shelf ESP32 or ESP32-S3 camera board, turning it into a MAVLink [optical flow](../sensor/optical_flow.md) sensor.
The board computes optical flow from a downward facing camera and sends [OPTICAL_FLOW_RAD](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD) messages to the flight controller over a UART, at up to 100 Hz (as measured on an AI-Thinker ESP32-CAM with an OV3660 camera).

The firmware does not measure distance.
A separate downward facing [distance sensor](../sensor/rangefinders.md) is required for optical flow navigation.

::: info
Dronaware Optical Flow is sold as a per-device licence.
The firmware is flashed and activated from a web browser at [dronaware.com/opticalflow](https://dronaware.com/opticalflow/).
:::

## Supported Boards

The following boards are currently available for activation.
The UART column lists the default ESP32 pins that carry MAVLink to the flight controller.
They can be customised during the flash session, as far as the board's free pins allow.

| Board                                                                    | SoC      | Camera         | Default UART TX / RX (GPIO) |
| ------------------------------------------------------------------------ | -------- | -------------- | --------------------------- |
| [AI-Thinker ESP32-CAM](https://docs.ai-thinker.com/en/esp32-cam)         | ESP32    | OV2640, OV3660 | 1 / 3                       |
| [LILYGO TTGO T-Journal](https://lilygo.cc/products/t-journal)            | ESP32    | OV2640         | 14 / 13                     |
| [Waveshare ESP32-S3-CAM](https://docs.waveshare.com/ESP32-S3-CAM-OVxxxx) | ESP32-S3 | GC0308, OV3660 | 43 / 44                     |

The complete list of boards, with their test status, is maintained in the _Device compatibility_ section of the [Dronaware documentation](https://dronaware.com/opticalflow/#documentation).

The firmware applies a lens calibration for each camera module, so the flow output is already in radians and [SENS_FLOW_SCALE](../advanced_config/parameter_reference.md#SENS_FLOW_SCALE) should normally be left at `1.0`.

## Wiring

Connect the board to a spare flight controller UART (for example `TELEM2`):

| ESP32 board | Flight controller |
| ----------- | ----------------- |
| TX          | RX                |
| RX          | TX                |
| GND         | GND               |
| 5V          | 5V                |

The firmware only transmits, so the flight controller TX to board RX line is unused (see [MAVLink Messages](#mavlink-messages)).
Both sides use 3.3 V logic, so no level shifter is needed.

::: tip
ESP32 camera boards draw current in short bursts while capturing frames.
A low-ESR electrolytic capacitor (for example 470 µF, 10 V or higher) across the board's 5V and GND input pins prevents brownout resets, particularly on the AI-Thinker ESP32-CAM.
:::

::: warning
On the AI-Thinker ESP32-CAM, the default MAVLink pins (1 / 3) are on UART0, which is also used by the USB-serial adapter.
When using the default pins, disconnect USB before connecting the board to the flight controller.
Do not power the board from both USB and the flight controller at the same time.
:::

## Mounting

Mount the board rigidly with the camera facing straight down and an unobstructed view of the ground.

Before installing the board, confirm the direction of its flow axes with the _Dronaware Testbench_ (opened automatically at the end of activation).
The Testbench reports which board edge must face the vehicle's front, and whether any flow axis is mirrored.
If it reports a mirrored axis, or cannot determine the direction, do not fly the board: `SENS_FLOW_ROT` corrects only yaw rotation, not a mirrored axis.
Contact Dronaware support instead.

Mount the board with that edge facing forwards and set [SENS_FLOW_ROT](../advanced_config/parameter_reference.md#SENS_FLOW_ROT) to `0` (no rotation).
If the board must be mounted in a different orientation, set `SENS_FLOW_ROT` to the matching yaw rotation.

The output must follow the direction convention in [Optical Flow > Setup](../sensor/optical_flow.md#setup) (forward movement gives + Y, right movement gives - X).

## PX4 Configuration

### MAVLink Port

Configure a dedicated [MAVLink instance](../peripherals/mavlink_peripherals.md) on the port the board is wired to:

| Parameter                                                                    | Value                                                       |
| ---------------------------------------------------------------------------- | ----------------------------------------------------------- |
| [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG)       | The port the board is connected to (for example `TELEM2`).  |
| [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE)           | `Custom` (no default streams).                              |
| [MAV_1_FORWARD](../advanced_config/parameter_reference.md#MAV_1_FORWARD)     | `0` (disabled).                                             |
| [MAV_1_FLOW_CTRL](../advanced_config/parameter_reference.md#MAV_1_FLOW_CTRL) | `0` (no hardware flow control).                             |
| [SER_TEL2_BAUD](../advanced_config/parameter_reference.md#SER_TEL2_BAUD)     | `115200` (use the baud parameter of the port you selected). |

`MAV_1_*` is used as an example; use whichever MAVLink instance is free on your vehicle.
Reboot after setting `MAV_1_CONFIG` so that the port's baud rate parameter appears.

### Estimator

Set the following parameters to enable optical flow fusion in [EKF2](../sensor/optical_flow.md#ekf2):

| Parameter                                                                                                                                                                                                                    | Value                                           |
| ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------- |
| [EKF2_OF_CTRL](../advanced_config/parameter_reference.md#EKF2_OF_CTRL)                                                                                                                                                       | `1` (enabled).                                  |
| [EKF2_OF_GYR_SRC](../advanced_config/parameter_reference.md#EKF2_OF_GYR_SRC)                                                                                                                                                 | `1` (Internal).                                 |
| [EKF2_OF_DELAY](../advanced_config/parameter_reference.md#EKF2_OF_DELAY)                                                                                                                                                     | `20` (ms).                                      |
| [SENS_FLOW_MAXR](../advanced_config/parameter_reference.md#SENS_FLOW_MAXR)                                                                                                                                                   | `1.0`.                                          |
| [EKF2_OF_POS_X](../advanced_config/parameter_reference.md#EKF2_OF_POS_X), [EKF2_OF_POS_Y](../advanced_config/parameter_reference.md#EKF2_OF_POS_Y), [EKF2_OF_POS_Z](../advanced_config/parameter_reference.md#EKF2_OF_POS_Z) | Camera position relative to the vehicle centre. |

The firmware leaves the gyro fields of `OPTICAL_FLOW_RAD` unset (`NaN`), so PX4 compensates rotation with its own gyro.
With `EKF2_OF_GYR_SRC=1`, EKF2 uses its IMU at the time of each flow measurement.
With the default (`0`, Auto), PX4 instead integrates its gyro separately for each message, and this under-reads the rotation because consecutive messages cover overlapping intervals (see [MAVLink Messages](#mavlink-messages)).
`EKF2_OF_DELAY=20` is the delay the sensor was flight tested with.
PX4's default (7 ms) suits DroneCAN flow nodes, so set `EKF2_OF_DELAY` explicitly and reboot.

`SENS_FLOW_MAXR=1.0` is the lowest value in the parameter's range.
PX4 then limits horizontal speed to about half the height above ground per second (about 0.5 m/s at 1 m), which keeps the flow rate within what the camera can measure.

Configure the downward [distance sensor](../sensor/rangefinders.md) as described in its own documentation.

## MAVLink Messages

The firmware uses MAVLink 2 at 115200 baud, 8 data bits, no parity, 1 stop bit.

- **Outgoing:** [OPTICAL_FLOW_RAD](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD) from system ID `42`, component ID `197` (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`).
  - Each message covers one camera frame interval (`integration_time_us`, about `50000` µs on 20 fps cameras and `44400` µs on the 22.5 fps OV2640).
    Several messages are sent per frame (about five at 20 fps, three on the OV2640), so consecutive intervals overlap.
  - `distance` is always `-1` (unknown); PX4 ignores it and uses the separate distance sensor.
  - `integrated_xgyro`, `integrated_ygyro` and `integrated_zgyro` are always `NaN`; PX4 compensates rotation with its own gyro (see [Estimator](#estimator)).
- **Incoming:** none. The firmware needs no data from the flight controller.

At 100 Hz, `OPTICAL_FLOW_RAD` uses about 5.6 kB/s of the roughly 11.5 kB/s available at 115200 baud.
The firmware sends at most 125 messages per second.

## Verification

1. In _QGroundControl_ open **Analyze Tools > MAVLink Inspector** and check that `OPTICAL_FLOW_RAD` is received at about 100 Hz (about 67 Hz with an OV2640 camera).
2. In the [MAVLink Shell](../debug/mavlink_shell.md), run `listener sensor_optical_flow` and check that `quality` rises above zero over a textured surface.
3. With the propellers removed, run `listener estimator_optical_flow_vel` in the MAVLink Shell, hold the vehicle over a textured surface and rotate it about its roll and pitch axes.
   `flow_rate_uncompensated` should follow `gyro_rate`, and `flow_rate_compensated` should stay near zero.
   Because the message gyro fields are `NaN`, the check in [Optical Flow > Scale Factor](../sensor/optical_flow.md#scale-factor) cannot be used.
   Do not adjust `SENS_FLOW_SCALE` based on `vehicle_optical_flow`, `vehicle_optical_flow_vel` or plots built from them: their gyro values come from PX4's per-message integration, which under-reads the rotation.
4. Fly the [MC_06 - Optical Flow](../test_cards/mc_06_optical_flow.md) test card.

## See Also

- [Optical Flow](../sensor/optical_flow.md)
- [Distance Sensors (Rangefinders)](../sensor/rangefinders.md)
- [MAVLink Peripherals](../peripherals/mavlink_peripherals.md)
- [Dronaware Optical Flow](https://dronaware.com/opticalflow/) (product page, setup guide and Testbench)
