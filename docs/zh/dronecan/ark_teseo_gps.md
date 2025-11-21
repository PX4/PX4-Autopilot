# ARK TESEO GPS

[ARK TESEO GPS](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-teseo-gps) is a made in the USA and NDAA-compliant [DroneCAN](index.md) [GNSS/GPS](../gps_compass/index.md) L1/L5 GPS, magnetometer, barometer, IMU, and buzzer module.

![ARK TESEO GPS](../../assets/hardware/gps/ark/ark_teseo_gps.jpg)

## 购买渠道

Order this module from:

- [ARK Electronics](https://arkelectron.com/product/ark-teseo-gps/) (US)

## Hardware Specifications

- [Open Source Schematic and BOM](https://github.com/ARK-Electronics/ARK_Teseo_GPS)
- 传感器
  - [ST TESEO LIV4F GPS](https://www.st.com/en/positioning/teseo-liv4f.html)
    - L1/L5 bands
    - Simultaneous multi-constellation and multi-band GNSS (GPS, Galileo, GLONASS, BeiDou, QZSS)
    - IRNSS constellation ready
    - -162 dBm tracking sensitivity
    - Submeter positioning accuracy
  - [ST IIS2MDC Magnetometer](https://www.st.com/en/mems-and-sensors/iis2mdc.html)
  - [Bosch BMP390 Barometer](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/pressure-sensors-bmp390.html)
  - [Invensense ICM-42688-P 6-Axis IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- STM32F412VGH6 MCU
- Dual Band (L1/L5) Helical GPS Antenna
- Two Pixhawk Standard CAN Connectors (4 Pin JST GH)
- I2C and Timepulse/PPS Connector (5 Pin JST GH)
- Pixhawk Standard Debug Connector (6 Pin JST SH)
- Power Requirements
  - 5V
  - 137mA
- LED Indicators
  - GPS Fix
  - RGB Status
- USA Built
- NDAA Compliant

## 针脚定义

### CAN - 4 Pin JST-GH

| Pin Number | Signal Name                | Voltage              |
| ---------- | -------------------------- | -------------------- |
| 1          | 5V                         | 5.0V |
| 2          | CAN_P | 5.0V |
| 3          | CAN_N | 5.0V |
| 4          | GND                        | GND                  |

### I2C + Timepulse - 5 Pin JST-GH

| Pin Number | Signal Name                                         | Voltage              |
| ---------- | --------------------------------------------------- | -------------------- |
| 1          | 5.0V Out (500mA) | 5.0V |
| 2          | I2C2_SCL                       | 3.3V |
| 3          | I2C2_SDA                       | 3.3V |
| 4          | TIMEPULSE                                           | 3.3V |
| 5          | GND                                                 | GND                  |

### Debug - 6 Pin JST-SH

| Pin Number | Signal Name                    | Voltage              |
| ---------- | ------------------------------ | -------------------- |
| 1          | 3.3V           | 3.3V |
| 2          | USART2_TX | 3.3V |
| 3          | USART2_RX | 3.3V |
| 4          | FMU_SWDIO | 3.3V |
| 5          | FMU_SWCLK | 3.3V |
| 6          | GND                            | GND                  |

## See Also

- [ARK TESEO GPS Documentation](https://arkelectron.gitbook.io/ark-documentation/sensors/ark-teseo-gps) (ARK Docs)
