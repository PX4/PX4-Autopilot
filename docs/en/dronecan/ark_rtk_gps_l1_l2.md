# ARK RTK GPS L1 L5

[ARK RTK GPS L1 L5](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-rtk-gps) is an open source [DroneCAN](index.md) [RTK GPS](../gps_compass/rtk_gps.md), [u-blox F9P](https://www.u-blox.com/en/product/zed-f9p-module), magnetometer, barometer, IMU, buzzer, and safety switch module.

![ARK RTK GPS L1 L5](../../assets/hardware/gps/ark/ark_rtk_gps_l1_l5.jpg)

## Where to Buy

Order this module from:

- [ARK Electronics](https://arkelectron.com/product/ark-rtk-gps-l1-l5/) (US)

## Hardware Specifications

- [Open Source Schematic and BOM](https://github.com/ARK-Electronics/ARK_RTK_GPS)
- Sensors
  - Ublox F9P GPS
    - Multi-band GNSS receiver delivers centimetre level accuracy in seconds
    - Concurrent reception of GPS, GLONASS, Galileo and BeiDou
    - Multi-band RTK with fast convergence times and reliable performance
    - High update rate for highly dynamic applications
    - Centimetre accuracy in a small and energy efficient module
    - Does not Support Moving Base for Heading
  - Bosch BMM150 Magnetometer
  - Bosch BMP388 Barometer
  - Invensense ICM-42688-P 6-Axis IMU
- STM32F412CEU6 MCU
- Safety Button
- Buzzer
- Two Pixhawk Standard CAN Connectors (4 Pin JST GH)
- F9P `UART 2` Connector
  - 3 Pin JST GH
  - TX, RX, GND
- Pixhawk Standard Debug Connector (6 Pin JST SH)
- LED Indicators
  - Safety LED
  - GPS Fix
  - RTK Status
  - RGB system status
- USA Built
- Power Requirements
  - 5V
  - 170mA Average
  - 180mA Max

## Hardware Setup

### Wiring

The ARK RTK GPS L1 L5 is connected to the CAN bus using a Pixhawk standard 4 pin JST GH cable. For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

### Mounting

The recommended mounting orientation is with the connectors on the board pointing towards the **back of vehicle**.

The sensor can be mounted anywhere on the frame, but you will need to specify its position, relative to vehicle centre of gravity, during [PX4 configuration](#px4-configuration).

## Firmware Setup

ARK RTK GPS L1 L5 runs the [PX4 cannode firmware](px4_cannode_fw.md). As such, it supports firmware update over the CAN bus and [dynamic node allocation](index.md#node-id-allocation).

ARK RTK GPS L1 L5 boards ship with recent firmware pre-installed, but if you want to build and flash the latest firmware yourself, refer to the [cannode firmware build instructions](px4_cannode_fw.md#building-the-firmware).

Firmware target: `ark_can-rtk-gps_default`
Bootloader target: `ark_can-rtk-gps_canbootloader`

## Flight Controller Setup

### Enabling DroneCAN

In order to use the ARK RTK GPS L1 L5, connect it to the Pixhawk CAN bus and enable the DroneCAN driver by setting parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)).

The steps are:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` or `3` and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Connect ARK RTK GPS L1 L5 CAN to the Pixhawk CAN.

Once enabled, the module will be detected on boot.
GPS data should arrive at 10Hz.

### PX4 Configuration

You need to set necessary [DroneCAN](index.md) parameters and define offsets if the sensor is not centred within the vehicle:

- Enable GPS yaw fusion by setting bit 3 of [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) to true.
- Enable GPS blending to ensure the heading is always published by setting [SENS_GPS_MASK](../advanced_config/parameter_reference.md#SENS_GPS_MASK) to 7 (all three bits checked).
- Enable [UAVCAN_SUB_GPS](../advanced_config/parameter_reference.md#UAVCAN_SUB_GPS), [UAVCAN_SUB_MAG](../advanced_config/parameter_reference.md#UAVCAN_SUB_MAG), and [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO).
- The parameters [EKF2_GPS_POS_X](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [EKF2_GPS_POS_Y](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y) and [EKF2_GPS_POS_Z](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z) can be set to account for the offset of the ARK RTK GPS L1 L5 from the vehicles centre of gravity.

### ARK RTK GPS L1 L5 Configuration

You may need to [configure the following parameters](../dronecan/index.md#qgc-cannode-parameter-configuration) on the ARK RTK GPS L1 L5 itself:

| Parameter                                                                                                | Description                                                                                                                           |
| -------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="CANNODE_NODE_ID"></a>[CANNODE_NODE_ID](../advanced_config/parameter_reference.md#CANNODE_NODE_ID) | CAN node ID (0 for dynamic allocation). If set to 0 (default), dynamic node allocation is used. Set to 1-127 to use a static node ID. |
| <a id="CANNODE_TERM"></a>[CANNODE_TERM](../advanced_config/parameter_reference.md#CANNODE_TERM)          | CAN built-in bus termination. Set to `1` if this is the last node on the CAN bus.                                                     |

### Setting Up Rover and Fixed Base

Position of the rover is established using RTCM messages from the RTK base module (the base module is connected to QGC, which sends the RTCM information to PX4 via MAVLink).

PX4 DroneCAN parameters:

- [UAVCAN_PUB_RTCM](../advanced_config/parameter_reference.md#UAVCAN_PUB_RTCM):
  - Makes PX4 publish RTCM messages ([RTCMStream](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#rtcmstream)) to the bus (which it gets from the RTK base module via QGC).

Rover module parameters (also [set using QGC](../dronecan/index.md#qgc-cannode-parameter-configuration)):

- [CANNODE_SUB_RTCM](../advanced_config/parameter_reference.md#CANNODE_SUB_RTCM) tells the rover that it should subscribe to [RTCMStream](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#rtcmstream) RTCM messages on the bus (from the moving base).

::: info
Use [UAVCAN_PUB_MBD](../advanced_config/parameter_reference.md#UAVCAN_PUB_MBD) and [CANNODE_SUB_MBD](../advanced_config/parameter_reference.md#CANNODE_SUB_MBD) instead if you want to implement moving base (see below) at the same time.
:::

For more information see [Rover and Fixed Base](../dronecan/index.md#rover-and-fixed-base) in the DroneCAN guide.

## LED Meanings

- The GPS status lights are located to the right of the connectors
  - Blinking green is GPS fix
  - Blinking blue is received corrections and RTK Float
  - Solid blue is RTK Fixed

- The CAN status lights are located top the left of the connectors
  - Slow blinking green is waiting for CAN connection
  - Fast blinking green is normal operation
  - Slow blinking green and blue is CAN enumeration
  - Fast blinking blue and red is firmware update in progress
  - Blinking red is error
    - If you see a red LED there is an error and you should check the following
      - Make sure the flight controller has an SD card installed
      - Make sure the ARK RTK GPS L1 L5 has `ark_can-rtk-gps_canbootloader` installed prior to flashing `ark_can-rtk-gps_default`
      - Remove binaries from the root and ufw directories of the SD card and try to build and flash again

### Updating Ublox F9P Module

ARK RTK GPS L1 L5 comes with the Ublox F9P module up to date with version 1.13 or newer. However, you can check the version and update the firmware if desired.

The steps are:

1. [Download u-center from u-blox.com](https://www.u-blox.com/en/product/u-center) and install on your PC (Windows only)
2. Open the [u-blox ZED-F9P website](https://www.u-blox.com/en/product/zed-f9p-module#tab-documentation-resources)
3. Scroll down and click on the "Show Legacy Documents" box
4. Scroll down again to Firmware Update and download your desired firmware (at least version 1.13 is needed)
5. While holding down the safety switch on the ARK RTK GPS L1 L5, connect it to power via one of its CAN ports and hold until all 3 LEDs blink rapidly
6. Connect the ARK RTK GPS L1 L5 to your PC via its debug port with a cable such as the Black Magic Probe or an FTDI
7. Open u-center, select the COM port for the ARK RTK GPS L1 L5 and connect
   ![U-Center Connect](../../assets/hardware/gps/ark/ark_rtk_gps_ucenter_connect.png)
8. Check the current firmware version by selecting View, Messages View, UBX, MON, VER
   ![Check Version](../../assets/hardware/gps/ark/ark_rtk_gps_ublox_version.png)
9. To update the firmware:
   1. Select Tools, Firmware Update
   2. The Firmware image field should be the .bin file downloaded from the u-blox ZED-F9P website
   3. Check the "Use this baudrate for update" checkbox and select 115200 from the drop-down
   4. Ensure the other checkboxes are as shown below
   5. Push the green GO button on the bottom left
   6. "Firmware Update SUCCESS" should be displayed if it updated successfully
      ![Firmware Update](../../assets/hardware/gps/ark/ark_rtk_gps_ublox_f9p_firmware_update.png)

## See Also

- [ARK RTK GPS L1 L5 Documentation](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-rtk-gps) (ARK Docs)
