# Holybro Durandal

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

_Durandal_<sup>&reg;</sup> is the latest update to the successful family of Holybro flight controllers.
It was designed and developed by Holybro.

![Durandal](../../assets/flight_controller/durandal/durandal_hero.jpg)

At high level, some of the key features are:

- Integrated temperature control for sensors.
- Powerful STM32H7 microcontroller running at 480MHz.
  2 MB of Flash memory and 1 MB of RAM.
- New sensors with higher temperature stability.
- Internal vibration isolation system.
- Dual high-performance, low-noise IMUs on board are designed for demanding stabilization applications.

A summary of the key features, [assembly](../assembly/quick_start_durandal.md), and [purchase](#purchase) links can be found below.

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## 总览

#### Technical Specifications

- Main FMU Processor: STM32H743
  - 32 Bit Arm ® Cortex® -M7, 480MHz, 2MB memory, 1MB RAM
- IO 处理器：STM32F100
  - 32 Bit Arm ® Cortex® -M3, 24MHz, 8KB SRAM
- On-board sensors
  - 加速度计 / 陀螺仪：ICM-20689
  - Accel/Gyro: BMI088 or ICM20602
  - Mag: IST8310
  - 气压计：MS5611
- GPS：ublox Neo-M8N GPS/GLONASS 接收器；集成磁力计 IST8310

#### 接口

- 8-13 PWM servo outputs (8 from IO, 5 from FMU)
- FMU上有6个专用PWM/Capture输入
- Dedicated R/C input for Spektrum / DSM
- Dedicated R/C input for CPPM and S.Bus
- Dedicated S.Bus servo output and analog / PWM RSSI input
- 5个通用串行口
  - 3 with full flow control
  - 1 with separate 1.5A current limit
- 3 个 I2C 接口
- 4路SPI总线
  - 1 internal high speed SPI sensor bus with 4 chip selects and 6 DRDYs
  - 1 internal low noise SPI bus dedicated for XXX
  - Barometer with 2 chip selects, no DRDYs
  - 1 internal SPI bus dedicated for FRAM
  - Supports temperature control located on sensor module
  - 1 external SPI buses
- Up to 2 CANBuses for dual CAN
  - Each CANBus has individual silent controls or ESC RX-MUX control
- 2个电池电流/电压模拟输入口
- 2 additional analog inputs

#### Electrical Data

- 电源模块输出：4.9~5.5V
- Max input voltage: 6V
- 最大电流感应：120A
- USB 电源输入：4.75~5.25V
- 伺服导轨输入电压：0~36V

#### Mechanical Data

- Dimensions: 80x45x20.5mm
- 重量：68.8g

#### 其它特性

- Operating temperature: ~40~85C
- Storage temperature: -40~85C
- CE
- FCC
- RoHS compliant (lead-free)

For more information see: [Durandal Technical Data Sheet](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Durandal_technical_data_sheet_90f8875d-8035-4632-a936-a0d178062077.pdf).

<a id="purchase"></a>

## 购买渠道

Order from [Holybro](https://holybro.com/collections/autopilot-flight-controllers/products/durandal).

<a id="connections"></a>

## Connections

The locations of ports/connections are shown here (and below in the [pinouts section](#pinouts)).

### Top

![Durandal - Top Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_top.jpg)

### Front

![Durandal - Front Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_front.jpg)

### Back

![Durandal - Back Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_back.jpg)

### Right

![Durandal - Right-side Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_right.jpg)

### Left

![Durandal - Left-side Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_left.jpg)

## 尺寸

All dimensions are in millimeters.

![Durandal Dimensions](../../assets/flight_controller/durandal/durandal_dimensions.jpg)

<!--
## Voltage Ratings

*Pixhawk 4* can be triple-redundant on the power supply if three power sources are supplied. The three power rails are: **POWER1**, **POWER2** and **USB**.

::: info
The output power rails **FMU PWM OUT** and **I/O PWM OUT** (0V to 36V) do not power the flight controller board (and are not powered by it).
You must supply power to one of **POWER1**, **POWER2** or **USB** or the board will be unpowered.
:::

**Normal Operation Maximum Ratings**

Under these conditions all power sources will be used in this order to power the system:
1. **POWER1** and **POWER2** inputs (4.9V to 5.5V)
1. **USB** input (4.75V to 5.25V)
-->

<!--
**Absolute Maximum Ratings**

Under these conditions the system will not draw any power (will not be operational), but will remain intact.
1. **POWER1** and **POWER2** inputs (operational range 4.1V to 5.7V, 0V to 10V undamaged)
1. **USB** input (operational range 4.1V to 5.7V, 0V to 6V undamaged)
1. Servo input: VDD_SERVO pin of **FMU PWM OUT** and **I/O PWM OUT** (0V to 42V undamaged)

-->

## 组装 / 设置

The [Durandal Wiring Quick Start](../assembly/quick_start_durandal.md) provides instructions on how to assemble required/important peripherals including GPS, Power Management Board etc.

## 编译固件

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make holybro_durandal-v1_default
```

## 串口映射

| UART   | 设备         | Port          |
| ------ | ---------- | ------------- |
| USART1 | /dev/ttyS0 | GPS1          |
| USART2 | /dev/ttyS1 | TELEM1        |
| USART3 | /dev/ttyS2 | TELEM2        |
| UART4  | /dev/ttyS3 | TELEM4/GPS2   |
| USART6 | /dev/ttyS4 | TELEM3        |
| UART7  | /dev/ttyS5 | Debug Console |
| UART8  | /dev/ttyS6 | PX4IO         |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

<a id="debug_port"></a>

## 调试接口

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) run on the _Debug Port_.

The pinouts and connector comply with the [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) interface defined in the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).

For wiring and debugging information see the above links.

:::info
No Debug port is exposed for the I/O board.
:::

## 外部设备

- [Digital Airspeed Sensor](https://store-drotek.com/793-digital-differential-airspeed-sensor-kit-.html)
- [Telemetry Radio Modules](../telemetry/index.md)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## 支持的平台/机身

Any multicopter / airplane / rover or boat that can be controlled with normal RC servos or Futaba S-Bus servos.

The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## 针脚定义

_Durandal_ pinouts are listed below.
These can also be downloaded from [here](https://holybro.com/collections/autopilot-flight-controllers/products/Durandal-Pinouts).

### Top Pinouts

![Durandal - Top Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_top.jpg)

### Front Pinouts

![Durandal - Front Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_front.jpg)

#### SUBS Out port

| 针脚                            | 信号                                                         | 电压                    |
| ----------------------------- | ---------------------------------------------------------- | --------------------- |
| 1（红）                          | -                                                          | -                     |
| 2 (yellow) | SBUS_OUT/RSSI_IN | +3.3V |
| 3 (black)  | GND                                                        | GND                   |

#### DSM RC port

| 针脚                            | 信号                           | 电压                    |
| ----------------------------- | ---------------------------- | --------------------- |
| 1（红）                          | VDD_3V3 | +3.3V |
| 2 (yellow) | DSM_IN  | +3.3V |
| 3 (black)  | GND                          | GND                   |

#### I2C A port

| 针脚                           | 信号   | 电压                    |
| ---------------------------- | ---- | --------------------- |
| 1（红）                         | VCC  | +5V                   |
| 2 (black) | SCL4 | +3.3V |
| 3 (black) | SDA4 | +3.3V |
| 4 (black) | GND  | GND                   |

#### CAN1 port

| 针脚                           | 信号    | 电压                    |
| ---------------------------- | ----- | --------------------- |
| 1（红）                         | VCC   | +5V                   |
| 2 (black) | CAN H | +3.3V |
| 3 (black) | CAN L | +3.3V |
| 4 (black) | GND   | GND                   |

<a id="gps"></a>

#### GPS 接口

| 针脚                            | 信号                                                          | 电压                    |
| ----------------------------- | ----------------------------------------------------------- | --------------------- |
| 1（红）                          | VCC                                                         | +5V                   |
| 2 (black)  | TX (out)                                 | +3.3V |
| 3 (black)  | RX (in)                                  | +3.3V |
| 4 (black)  | SCL1                                                        | +3.3V |
| 5 (black)  | SDA1                                                        | +3.3V |
| 6 (black)  | SAFETY_SWITCH                          | +3.3V |
| 7 (black)  | SAFETY_SWITCH_LED | +3.3V |
| 8 (black)  | VDD_3V3                                | +3.3V |
| 9 (black)  | BUZZER                                                      | +5V                   |
| 10 (black) | GND                                                         | GND                   |

<a id="telem4_i2cb"></a>

#### TELEM4 I2CB ports

| 针脚                           | 信号                          | 电压                    |
| ---------------------------- | --------------------------- | --------------------- |
| 1（红）                         | VCC                         | +5V                   |
| 2 (black) | TX (out) | +3.3V |
| 3 (black) | RX (in)  | -                     |
| 4 (black) | SCL2                        | -                     |
| 5 (black) | SDA2                        | +3.3V |
| 6 (black) | GND                         | GND                   |

<a id="telem1_2_3"></a>

#### TELEM3, TELEM2, TELEM1 port

| 针脚                           | 信号                          | 电压                    |
| ---------------------------- | --------------------------- | --------------------- |
| 1（红）                         | VCC                         | +5V                   |
| 2 (black) | TX (out) | +3.3V |
| 3 (black) | RX (in)  | +3.3V |
| 4 (black) | CTS（输入）                     | +3.3V |
| 5 (black) | RTS（输出）                     | +3.3V |
| 6 (black) | GND                         | GND                   |

<a id="power"></a>

#### POWER port

| 针脚                           | 信号  | 电压                    |
| ---------------------------- | --- | --------------------- |
| 1（红）                         | VCC | +5V                   |
| 2 (black) | VCC | +5V                   |
| 3 (black) | 电流  | +3.3V |
| 4 (black) | 电压  | +3.3V |
| 5 (black) | GND | GND                   |
| 6 (black) | GND | GND                   |

### Back Pinouts

![Durandal - Back Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_back.jpg)

#### MAIN Out

| 针脚 | 信号                          | 电压                    | +                              | -   |
| -- | --------------------------- | --------------------- | ------------------------------ | --- |
| 1  | IO_CH1 | +3.3V | VDD_SERVO | GND |
| 2  | IO_CH2 | +3.3V | VDD_SERVO | GND |
| 3  | IO_CH3 | +3.3V | VDD_SERVO | GND |
| 4  | IO_CH4 | +3.3V | VDD_SERVO | GND |
| 5  | IO_CH5 | +3.3V | VDD_SERVO | GND |
| 6  | IO_CH6 | +3.3V | VDD_SERVO | GND |
| 7  | IO_CH7 | +3.3V | VDD_SERVO | GND |
| 8  | IO_CH8 | +3.3V | VDD_SERVO | GND |

#### AUX Out

| 针脚 | 信号                           | 电压                    | +                              | -   |
| -- | ---------------------------- | --------------------- | ------------------------------ | --- |
| 1  | FMU_CH1 | +3.3V | VDD_SERVO | GND |
| 2  | FMU_CH2 | +3.3V | VDD_SERVO | GND |
| 3  | FMU_CH3 | +3.3V | VDD_SERVO | GND |
| 4  | FMU_CH4 | +3.3V | VDD_SERVO | GND |
| 5  | FMU_CH5 | +3.3V | VDD_SERVO | GND |

#### RC IN

| 针脚 | 信号                                                       | 电压                    |
| -- | -------------------------------------------------------- | --------------------- |
| S  | SBUS_IN/PPM_IN | +3.3V |
| +  | VCC                                                      | +5V                   |
| -  | GND                                                      | GND                   |

### Right-side Pinouts

![Durandal - Right-side Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_right.jpg)

#### CAN2 port

| 针脚                           | 信号    | 电压                    |
| ---------------------------- | ----- | --------------------- |
| 1（红）                         | VCC   | +5V                   |
| 2 (black) | CAN H | +3.3V |
| 3 (black) | CAN L | +3.3V |
| 4 (black) | GND   | GND                   |

#### CAP & ADC IN port

| 针脚                            | 信号                                                     | 电压                                       |
| ----------------------------- | ------------------------------------------------------ | ---------------------------------------- |
| 1（红）                          | VCC                                                    | +5V                                      |
| 2 (black)  | FMU_CAP6                          | +3.3V                    |
| 3 (black)  | FMU_CAP5                          | +3.3V                    |
| 4 (black)  | FMU_CAP4                          | +3.3V                    |
| 5 (black)  | FMU_CAP3                          | +3.3V                    |
| 6 (black)  | FMU_CAP2                          | +3.3V                    |
| 7 (black)  | FMU_CAP1                          | +3.3V                    |
| 8 (black)  | ADC1_SPARE_1 | +3.3V [++](#warn_sensor) |
| 9 (black)  | ADC1_SPARE_2 | +6.6V [++](#warn_sensor) |
| 10 (black) | GND                                                    | GND                                      |

<a id="warn_sensor"></a>

:::warning
\++ Sensors connected to pins 8, 9 must not send a signal exceeding the indicated voltage.
:::

### Left-side Pinouts

![Durandal - Left-side Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_left.jpg)

<a id="debug_port"></a>

#### DEBUG port

| 针脚                           | 信号    | 电压                    |
| ---------------------------- | ----- | --------------------- |
| 1（红）                         | VT    | +3.3V |
| 2 (black) | TX    | +3.3V |
| 3 (black) | RX    | +3.3V |
| 4 (black) | SWDIO | +3.3V |
| 5 (black) | SWCLK | +3.3V |
| 6 (black) | GND   | GND                   |

#### SPI port

| 针脚                           | 信号   | 电压                    |
| ---------------------------- | ---- | --------------------- |
| 1（红）                         | VCC  | +5V                   |
| 2 (black) | SCK  | +3.3V |
| 3 (black) | MISO | +3.3V |
| 4 (black) | MOSI | +3.3V |
| 5 (black) | CS1  | +3.3V |
| 6 (black) | CS2  | +3.3V |
| 7 (black) | GND  | GND                   |

#### USB port

| 针脚                           | 信号   | 电压                    |
| ---------------------------- | ---- | --------------------- |
| 1（红）                         | VBUS | +5V                   |
| 2 (black) | DM   | +3.3V |
| 3 (black) | DP   | +3.3V |
| 4 (black) | GND  | GND                   |

## 更多信息

- [Durandal Wiring QuickStart](../assembly/quick_start_durandal.md)
- [Durandal Technical Data Sheet](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Durandal_technical_data_sheet_90f8875d-8035-4632-a936-a0d178062077.pdf)
- [Durandal Pinouts](https://holybro.com/collections/autopilot-flight-controllers/products/Durandal-Pinouts) (Holybro)
