# Trimble MB-Two

The [Trimble MB-Two RTK GPS receiver](https://oemgnss.trimble.com/en/products/receiver-modules/mb-two) is a high-end, dual-frequency [RTK GPS module](../gps_compass/rtk_gps.md) that can be configured as either base or rover.

![MB-Two Hero image](../../assets/hardware/gps/rtk_trimble_two_gnss_hero.jpg)

## Required Firmware Options

The following firmware options need to be selected when buying the device:

- \[X\] \[2\] \[N\] \[G\] \[W\] \[Y\] \[J\] for 20Hz position updates and RTK support, 1cm horizontal and 2cm vertical position accuracy
- \[L\] LBAND
- \[D\] DUO - Dual Antenna Heading
- \[B\] BEIDOU + \[O\] GALILEO, if needed

## Antennas and Cable

The Trimble MB-Two requires two dual-frequency (L1/L2) antennas.
A good example is the [Maxtenna M1227HCT-A2-SMA](https://www.maxtena.com/products/helicore/m1227hct-a2-sma/)
(which can be bought, for instance, from [Farnell](https://uk.farnell.com/maxtena/m1227hct-a2-sma/antenna-1-217-1-25-1-565-1-61ghz/dp/2484959)).

The antenna connector type on the device is MMCX.
Suitable cables for the above antennas (SMA connector) can be found here:

- [30 cm version](https://www.digikey.com/products/en?mpart=415-0073-012&v=24)
- [45 cm version](https://www.digikey.com/products/en?mpart=415-0073-018&v=24)

## 接线和连接

The Trimble MB-Two is connected to a UART on the flight controller (GPS port) for data.

To power the module you will need a separate 3.3V power supply (the maximum consumption is 360mA).

:::info
The module cannot be powered from a Pixhawk.
:::

The pins on the 28-pin connector are numbered as shown below:

![MB-Two Pinout](../../assets/hardware/gps/rtk_trimble_two_gnss_pinouts.jpg)

| 针脚 | 参数名                      | 描述                                                   |
| -- | ------------------------ | ---------------------------------------------------- |
| 6  | Vcc 3.3V | Power supply                                         |
| 14 | GND                      | Connect to power the supply and GND of the Autopilot |
| 15 | TXD1                     | Connect to RX of the Autopilot                       |
| 16 | RXD1                     | Connect to TX of the Autopilot                       |

## 配置

First set the GPS protocol to Trimble ([GPS_x_PROTOCOL=3](../advanced_config/parameter_reference.md#GPS_1_PROTOCOL)).

[Configure the serial port](../peripherals/serial_configuration.md) on which the Trimple will run using [GPS_1_CONFIG](../advanced_config/parameter_reference.md#GPS_1_CONFIG), and set the baud rate to 115200 using [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD).

:::info
PX4 doesn't use the MB-Two's dual-antenna heading.
:::
