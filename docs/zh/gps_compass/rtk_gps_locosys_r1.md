# LOCOSYS Hawk R1 RTK GPS

<Badge type="tip" text="PX4 v1.13" />

The [LOCOSYS Hawk R1](https://www.locosystech.com/en/product/hawk-r1.html) is a dual-frequency [RTK GPS module](../gps_compass/rtk_gps.md) receiver designed for compatibility with Pixhawk.

The module can act as an RTK GPS rover when installed on the aircraft.

The receiver is capable of concurrently tracking all global civil navigation systems, including GPS, GLONASS, GALILEO, BEIDOU and QZSS.
It acquires both L1 and L5 signals at the same time while providing the centimeter-level RTK positioning accuracy.

The built-in lightweight helical antenna enhances RTK positioning stability.
The fast time-to-first-fix, RTK convergence, superior sensitivity, low power consumption make it a better choice for Pixhawk-based platform UAVs.

:::info
This module does not have a compass.
For an equivalent GPS module with a compass try: [LOCOSYS Hawk R2](../gps_compass/rtk_gps_locosys_r2.md).
:::

## Main Features

- Concurrent reception of L1 and L5 band signals
- Support GPS, GLONASS, BEIDOU, GALILEO, QZSS
- Capable of SBAS (WAAS, EGNOS, MSAS, GAGAN)
- Support 135-channel GNSS
- Fast TTFF at low signal level
- Free hybrid ephemeris prediction to achieve faster cold start
- Default 5Hz, up to 10 Hz update rate (SBAS support 5Hz only).
- Build-in super capacitor to reserve system data for rapid satellite acquisition

![LOCOSYS Hawk R1](../../assets/hardware/gps/locosys_hawk_a1/locosys_hawk_a1_gps.png)

## 购买渠道

- [LOCOSYS Hawk R1](https://www.locosystech.com/en/product/hawk-r1.html)

## Kit Contents

An RTK GPS kit includes:

- 1x GPS Module
- 1x Helix antenna
- 1x 6-pin JST-GH

## 配置

RTK setup and use on PX4 via _QGroundControl_ is largely plug and play (see [RTK GPS](../gps_compass/rtk_gps.md) for more information).
Connect your Hawk R1 to the `GPS2` port on compatible Pixhawk boards (preferred, though you can use any other unused UART port).

For the aircraft, you should set the parameter [SER_GPS2_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD) to 230400 8N1 to ensure that PX4 configures the correct baudrate.

## 接线和连接

Hawk R1 RTK GPS comes with an 6 pin JST-GH connector that can be plugged into a Pixhawk autopilot.

### 针脚定义

LOCOSYS GPS pinout is provided below.

| 针脚 | Hawk R1 GPS                 |
| -- | --------------------------- |
| 1  | VCC_5V |
| 2  | GPS_RX |
| 3  | GPS_TX |
| 4  | Null                        |
| 5  | Null                        |
| 6  | Null                        |
| 7  | Null                        |
| 8  | Null                        |
| 9  | GND                         |

## Status LEDs

| 颜色  | 参数名             | 描述                                 |
| --- | --------------- | ---------------------------------- |
| 绿色  | TX Indicator    | GNSS Data transmission             |
| Red | Power Indicator | 电源                                 |
| 蓝色  | PPS             | Precise Positioning Service active |

![Hawk A1 LEDs](../../assets/hardware/gps/locosys_hawk_a1/locosys_hawk_a1_leds.png)

## 技术规范

- Frequency
  - GPS/QZSS: L1 C/A, L5C
  - GLONASS: L1OF
  - BEIDOU: B1I, B2a
  - GALILEO: E1, E5a
- 135 Channels support
- Up to 10 Hz update rate (default to 5Hz)
- Acquisition Time
  - Hot start (Open Sky) in 2 seconds
  - Cold Start (Open Sky) in 28 seconds without AGPS
- PPS with 100ms pulse width, 1.8Vdc
- External, active Helix antenna
  - SMA connector
- UBlox Protocol Support
  - U5Hz:UBX-NAV-PVT,UBX-NAV-DOP
  - 1Hz: UBX-NAV-TIMEGPS
- Connectivity:
  - 6-pin JST-GH UART/I2C (Pixhawk compatible)
- 电源：
  - DC supply voltage 3.3V ~ 5.0V input
  - Power consumption <1W

## More Information

More information can be found on [LOCOSYS Hawk R1](https://www.locosystech.com/en/product/hawk-r1.html)
