# ARK G5 RTK GPS

[ARK G5 RTK GPS](https://arkelectron.com/product/ark-g5-rtk-gps/) is made in the USA NDAA compliant [DroneCAN](index.md)  quad-band  [RTK GPS](../gps_compass/rtk_gps.md), [ Septentrio mosaic-G5 P3 Ultra-compact high-precision GPS/GNSS receiver module](https://www.u-blox.com/en/product/zed-x20p-module), magnetometer, barometer, IMU, and buzzer module.

![ARK G5 RTK GPS](../../assets/hardware/gps/ark/ark_g5_rtk_gps.png)

## Where to Buy

Order this module from:

- [ARK Electronics](https://arkelectron.com/product/ark-g5-rtk-gps/) (US)

## Hardware Specifications

- [DroneCAN](index.md) RTK GNSS, Magnetometer, Barometer, IMU, and Buzzer Module
- [Supports Dronecan Firmware Updating](https://docs.px4.io/main/en/dronecan/#firmware-update)
- Sensors
  - [Septentrio mosaic-G5 P3 Ultra-compact high-precision GPS/GNSS receiver module](https://www.septentrio.com/en/products/gnss-receivers/gnss-receiver-modules/mosaic-G5-P3)
    - All-band all constellation GNSS receiver
    - All-in-view satellite tracking: multi-constellation, quad-band GNSS module receiver
    - Full raw data with positioning measurements and Galileo HAS positioning service compatibility
    - Best-in-class RTK cm-level positioning accuracy
    - Advanced GNSS+ algorithms
    - 20Hz update rate
  - [ST IIS2MDC Magnetometer](https://www.st.com/en/mems-and-sensors/iis2mdc.html)
  - [Bosch BMP390 Barometer](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390/)
  - [Invensense ICM-42688-P 6-Axis IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- STM32F412VGH6 MCU
- Safety Button
- Buzzer
- Two Pixhawk Standard CAN Connectors (4 Pin JST GH)
- G5 “UART 2” Connector
  - 4 Pin JST GH
  - TX, RX, PPS, GND
- G5 USB C
- Pixhawk Standard Debug Connector (6 Pin JST SH)
- LED Indicators
  - GPS Fix
  - RTK Status
  - RGB system status
- USA Built
- NDAA Compliant
- Power Requirements
  - 5V
    - 270mA
- Dimensions
  - Without Antenna
    - 48.0mm x 40.0mm x 15.4mm
    - 13.0g
  - With Antenna
    - 48.0mm x 40.0mm x 51.0mm
    - 43.5g
- Includes
  - 4 Pin Pixhawk Standard CAN Cable
  - Full-Frequency Helical GPS Antenna

## Hardware Setup

### Wiring

The ARK G5 RTK GPS is connected to the CAN bus using a Pixhawk standard 4 pin JST GH cable. For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

### Mounting

The recommended mounting orientation is with the connectors on the board pointing towards the **back of vehicle**.

The sensor can be mounted anywhere on the frame, but you will need to specify its position, relative to vehicle centre of gravity, during [PX4 configuration](#px4-configuration).

## Firmware Setup

The Septentrio G5 module firmware can be updated using the Septentrio [RxTools](https://www.septentrio.com/en/products/gps-gnss-receiver-software/rxtools) application.

## Flight Controller Setup

### Enabling DroneCAN

In order to use the ARK G5 RTK GPS, connect it to the Pixhawk CAN bus and enable the DroneCAN driver by setting parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)).

The steps are:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` or `3` and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Connect ARK G5 RTK GPS CAN to the Pixhawk CAN.

Once enabled, the module will be detected on boot.

### PX4 Configuration

You need to set necessary [DroneCAN](index.md) parameters and define offsets if the sensor is not centred within the vehicle:

- Enable GPS yaw fusion by setting bit 3 of [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) to true.
- Enable GPS blending to ensure the heading is always published by setting [SENS_GPS_MASK](../advanced_config/parameter_reference.md#SENS_GPS_MASK) to 7 (all three bits checked).
- Enable [UAVCAN_SUB_GPS](../advanced_config/parameter_reference.md#UAVCAN_SUB_GPS), [UAVCAN_SUB_MAG](../advanced_config/parameter_reference.md#UAVCAN_SUB_MAG), and [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO).
- The parameters [EKF2_GPS_POS_X](../advanced_config/parameter_reference.md#EKF2_GPS_POS_X), [EKF2_GPS_POS_Y](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Y) and [EKF2_GPS_POS_Z](../advanced_config/parameter_reference.md#EKF2_GPS_POS_Z) can be set to account for the offset of the ARK G5 RTK GPS from the vehicles centre of gravity.

## LED Meanings

- The GPS status lights are located to the right of the connectors
  - Blinking green is GPS fix
  - Blinking blue is received corrections and RTK Float
  - Solid blue is RTK Fixed

## See Also

- [ARK G5 RTK GPS Documentation](https://docs.arkelectron.com/gps/ark-g5-rtk-gps) (ARK Docs)
