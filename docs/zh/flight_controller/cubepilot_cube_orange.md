# CubePilot Cube Orange Flight Controller

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://cubepilot.org/#/home) for hardware support or compliance issues.
:::

The [Cube Orange](https://www.cubepilot.com/#/cube/features) flight controller is a flexible autopilot intended primarily for manufacturers of commercial systems.

![Cube Orange](../../assets/flight_controller/cube/orange/cube_orange_hero.jpg)

The controller is designed to be used with a domain-specific carrier board in order to reduce the wiring, improve reliability, and ease of assembly.
For example, a carrier board for a commercial inspection vehicle might include connections for a companion computer, while a carrier board for a racer could includes ESCs for the frame of the vehicle.

The ADS-B carrier board includes a customized 1090MHz ADSB-In receiver from uAvionix.
This provides attitude and location of commercial manned aircraft within the range of Cube.
This is automatically configured and enabled in the default PX4 firmware.

Cube includes vibration isolation on two of the IMU's, with a third fixed IMU as a reference / backup.

:::tip
The manufacturer [Cube Docs](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview) contain detailed information, including an overview of the [Differences between Cube Colours](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview#differences-between-cube-colours).
:::

## 主要特性

- 32bit STM32H753VI (32bit [ARM Cortex M7](https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M7), 400 MHz, Flash 2MB, RAM 1MB).
- 32 bit STM32F103 failsafe co-processor
- 14 PWM / Servo outputs (8 with failsafe and manual override, 6 auxiliary, high-power compatible)
- Abundant connectivity options for additional peripherals (UART, I2C, CAN)
- Integrated backup system for in-flight recovery and manual override with dedicated processor and stand-alone power supply (fixed-wing use)
- Backup system integrates mixing, providing consistent autopilot and manual override mixing modes (fixed-wing use)
- Redundant power supply inputs and automatic failover
- External safety switch
- Multicolor LED main visual indicator
- High-power, multi-tone piezo audio indicator
- microSD card for high-rate logging over extended periods of time

<a id="stores"></a>

## 购买渠道

- [Reseller list](https://www.cubepilot.com/#/reseller/list)

## 组装

[Cube Wiring Quickstart](../assembly/quick_start_cube.md)

## 产品规格

- **Processor:**
  - STM32H753VI (32bit [ARM Cortex M7](https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M7))
  - 400 MHz
  - 1 MB RAM
  - 2 MB Flash \(fully accessible\)
- **Failsafe co-processor:** <!-- inconsistent info on failsafe processor: 32 bit STM32F103 failsafe co-processor http://www.proficnc.com/all-products/191-pixhawk2-suite.html -->
  - STM32F103 (32bit _ARM Cortex-M3_)
  - 24 MHz
  - 8 KB SRAM
- **Sensors:** (all connected via SPI)
  - **Accelerometer:** (3) ICM20948, ICM20649, ICM20602
  - **Gyroscope:** (3) ICM20948, ICM20649, ICM20602
  - **Compass:** (1) ICM20948
  - **Barometric Pressure Sensor:** (2) MS5611
- **Operating Conditions:**
  - **Operating Temp:** -10C to 55C
  - **IP rating/Waterproofing:** Not waterproof
  - **Servo rail input voltage:** 3.3V / 5V
  - **USB port input:**
    - Voltage: 4V - 5.7V
    - Rated current: 250 mA
  - **POWER:**
    - Input voltage: 4.1V - 5.7V
    - Rated input current: 2.5A
    - Rated input/output power: 14W
- **Dimensions:**
  - **Cube:** 38.25mm x 38.25mm x 22.3mm
  - **Carrier:** 94.5mm x 44.3mm x 17.3mm
- **Interfaces**
  - IO Ports: 14 PWM servo outputs (8 from IO, 6 from FMU)
  - 5x UART (serial ports), one high-power capable, 2x with HW flow control
  - 2x CAN (one with internal 3.3V transceiver, one on expansion connector)
  - **R/C inputs:**
    - Spektrum DSM / DSM2 / DSM-X® Satellite compatible input
    - Futaba S.BUS® compatible input and output
    - PPM-SUM signal input
  - RSSI (PWM or voltage) input
  - I2C
  - SPI
  - 3.3v ADC input
  - Internal microUSB port and external microUSB port extension

## Ports

### Top-Side (GPS, TELEM etc)

![Cube Ports - Top (GPS, TELEM etc) and Main/AUX](../../assets/flight_controller/cube/cube_ports_top_main.jpg)

## 针脚定义

#### TELEM1，TELEM2 接口

| 针脚   | 信号                           | 电压                    |
| ---- | ---------------------------- | --------------------- |
| 1（红） | VCC                          | +5V                   |
| 2    | TX (OUT)  | +3.3V |
| 3    | RX (IN)   | +3.3V |
| 4（黑） | CTS (IN)  | +3.3V |
| 6    | RTS (OUT) | +3.3V |
| 6    | GND                          | GND                   |

#### GPS1 port

| 针脚                         | 信号                          | 电压                    |
| -------------------------- | --------------------------- | --------------------- |
| 1（红）                       | VCC                         | +5V                   |
| 2                          | TX (OUT) | +3.3V |
| 3                          | RX (IN)  | +3.3V |
| 4（黑）                       | SCL I2C2                    | +3.3V |
| 6                          | SDA I2C2                    | +3.3V |
| 6                          | Safety Button               | GND                   |
| 7                          | Button LED                  | GND                   |
| 8 (blk) | GND                         | GND                   |

<!-- check is i2c2 -->

#### GPS2 port

| 针脚   | 信号                          | 电压                    |
| ---- | --------------------------- | --------------------- |
| 1（红） | VCC                         | +5V                   |
| 2    | TX (OUT) | +3.3V |
| 3    | RX (IN)  | +3.3V |
| 4（黑） | SCL I2C1                    | +3.3V |
| 6    | SDA I2C1                    | +3.3V |
| 6    | GND                         | GND                   |

#### ADC

| 针脚   | 信号     | 电压                          |
| ---- | ------ | --------------------------- |
| 1（红） | VCC    | +5V                         |
| 2    | ADC IN | up to +6.6V |
| 3    | GND    | GND                         |

#### I2C

| 针脚   | 信号  | 电压                                                |
| ---- | --- | ------------------------------------------------- |
| 1（红） | VCC | +5V                                               |
| 2    | SCL | +3.3 (pullups) |
| 3    | SDA | +3.3 (pullups) |
| 4（黑） | GND | GND                                               |

#### CAN1 & CAN2

| 针脚   | 信号                         | 电压   |
| ---- | -------------------------- | ---- |
| 1（红） | VCC                        | +5V  |
| 2    | CAN_H | +12V |
| 3    | CAN_L | +12V |
| 4（黑） | GND                        | GND  |

#### POWER1 & POWER2

| 针脚                         | 信号              | 电压                    |
| -------------------------- | --------------- | --------------------- |
| 1（红）                       | VCC             | +5V                   |
| 2 (red) | VCC             | +5V                   |
| 3                          | CURRENT sensing | +3.3V |
| 4（黑）                       | VOLTAGE sensing | +3.3V |
| 6                          | GND             | GND                   |
| 6                          | GND             | GND                   |

#### USB

| 针脚   | 信号                           | 电压                    |
| ---- | ---------------------------- | --------------------- |
| 1（红） | VCC                          | +5V                   |
| 2    | OTG_DP1 | +3.3V |
| 3    | OTG_DM1 | +3.3V |
| 4（黑） | GND                          | GND                   |
| 6    | BUZZER                       | Battery voltage       |
| 6    | FMU Error LED                |                       |

#### SPKT

| 针脚                         | 信号  | 电压                    |
| -------------------------- | --- | --------------------- |
| 1 (blk) | IN  |                       |
| 2                          | GND | GND                   |
| 3 (red) | OUT | +3.3V |

#### TELEM1, TELEM2

| 针脚   | 信号                           | 电压                          |
| ---- | ---------------------------- | --------------------------- |
| 1（红） | VCC                          | +5V                         |
| 2    | TX (OUT)  | +3.3V to 5V |
| 3    | RX (IN)   | +3.3V to 5V |
| 4（黑） | CTS (OUT) | +3.3V to 5V |
| 6    | RTS (IN)  | +3.3V to 5V |
| 6    | GND                          | GND                         |

## 串口映射

| UART   | 设备         | Port                                     |
| ------ | ---------- | ---------------------------------------- |
| USART2 | /dev/ttyS0 | TELEM1 (flow control) |
| USART3 | /dev/ttyS1 | TELEM2 (flow control) |
| UART4  | /dev/ttyS2 | GPS1                                     |
| USART6 | /dev/ttyS3 | PX4IO                                    |
| UART7  | /dev/ttyS4 | CONSOLE/ADSB-IN                          |
| UART8  | /dev/ttyS5 | GPS2                                     |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/boards/cubepilot/cubeorange/default.px4board -->

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/boards/cubepilot/cubeorange/nuttx-config/nsh/defconfig#L188-L197 -->

### USB/SDCard Ports

![Cube USB/SDCard Ports](../../assets/flight_controller/cube/cube_ports_usb_sdcard.jpg)

## 编译固件

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target, open up the terminal and enter:

```
make cubepilot_cubeorange
```

## 原理图

Board schematics and other documentation can be found here: [The Cube Project](https://github.com/proficnc/The-Cube).

## Further Information/Documentation

- [Cube Wiring Quickstart](../assembly/quick_start_cube.md)
- Cube Docs (Manufacturer):
  - [Cube Module Overview](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview)
  - [Cube User Manual](https://docs.cubepilot.org/user-guides/autopilot/the-cube-user-manual)
  - [Mini Carrier Board](https://docs.cubepilot.org/user-guides/carrier-boards/mini-carrier-board)
