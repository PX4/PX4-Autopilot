# Femtones MINI2 Receiver

[MINI2 Receiver](http://www.femtomes.com/#/MiniII?type=0) is an RTK GPS receiver that delivers high-rate and reliable RTK initialization for centimeter level positioning.
It is intended for applications that require high-precision positioning (e.g. navigation and mapping, etc.).

The receiver is connected to PX4 via a serial port (UART) and may be configured over Ethernet using a standard web browser.

![MINI II Receiver](../../assets/hardware/gps/rtk_fem_miniII_receiver.jpg)

::: info
PX4 drivers for Ethernet, CAN and USB are under development.
:::

## Required Firmware Options

The following firmware options need to be selected when buying the device:

- 5Hz, 10Hz, 20Hz
- INS
- HEADING
- OBS
- RTK
- BASE

## Where to Buy

Contact [Femtones](http://www.femtomes.com/) directly for sales quote:

- **Email:** [sales@femtomes.com](mailto:sales@femtomes.com)
- **Telephone:** +86-10-53779838

## Functional Ports

![MINI II 1](../../assets/hardware/gps/rtk_fem_miniII_1.jpg)

## Wiring and Connections

The [MINI2 Receiver](http://www.femtomes.com) is connected to a UART on the flight controller (GPS port) for data.
To power the module you will need a separate 12V power supply.
The pins on the 12-pin connector are numbered as shown below.

![MINI_II_2](../../assets/hardware/gps/rtk_fem_miniII_2.jpg)

## Configuration

For heading estimation the two antennas need to be on the same level and at least 30 cm apart from each other.
The direction that they are facing does not matter as it can be configured with the [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET) parameter.

Configure the serial port on which the [MINI2 Receiver](http://www.femtomes.com/#/MiniII?type=0) will run using [GPS_1_CONFIG](../advanced_config/parameter_reference.md#GPS_1_CONFIG), and set the baud rate to 115200 using [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD).

Once configured the receiver is used in the same way as any other [RTK GPS](../gps_compass/rtk_gps.md) (i.e. with respect to the Survey-in process).

## Additional Information

The MINI2 incorporates the following components:

- [FB672](http://www.femtomes.com/#/FB672): Compact, dual antenna, dual frequency GNSS OEM board (delivers centimeter accurate position and precise heading).

  ![FB672](../../assets/hardware/gps/rtk_fem_fb_1.jpg)

- [FB6A0](http://www.femtomes.com/#/FB6A0): Compact, quadruple frequency GNSS OEM board (delivers centimeter-accurate positioning)

  ![FB6A0](../../assets/hardware/gps/rtk_fem_fb_2.jpg)

Detailed product instructions can be obtained on the official website or by contacting us.
