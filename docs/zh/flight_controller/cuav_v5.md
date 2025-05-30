# CUAV v5 (Discontinued)

<Badge type="info" text="Discontinued" />

:::warning
This flight controller has been [discontinued](../flight_controller/autopilot_experimental.md) and is no longer commercially available.
:::

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

_CUAV v5_<sup>&reg;</sup> (previously "Pixhack v5") is an advanced autopilot designed and made by CUAV<sup>&reg;</sup>.
The board is based on the [Pixhawk-project](https://pixhawk.org/) **FMUv5** open hardware design.
It runs PX4 on the [NuttX](https://nuttx.apache.org/) OS, and is fully compatible with PX4 firmware.
It is intended primarily for academic and commercial developers.

![CUAV v5](../../assets/flight_controller/cuav_v5/pixhack_v5.jpg)

## 总览

- 主处理器：STM32F765
  - 32 位 Arm® Cortex®-M7，216MHz，2MB 储存，512KB RAM

- IO 处理器：STM32F100
  - 32 位 Arm® Cortex®-M3，24MHz，8KB SRAM

- 内置传感器：

  - 加速度计/陀螺仪：ICM-20689
  - 加速度计/陀螺仪：BMI055
  - 磁力计：IST8310
  - 气压计：MS5611

- 接口：
  - 14路PWM输出 (6路来自FMU, 8路来自 IO)
  - FMU上有3个专用PWM/Capture输入
  - CPPM专用的RC输入
  - Dedicated R/C input for PPM and S.Bus
  - 电平/PWM RSSI输入
  - S.BUS伺服输出
  - 5个通用串行口
  - 4路I2C总线
  - 4路SPI总线
  - 2 CANBuses with serial ESC
  - 2个电池电流/电压模拟输入口

- 电源系统
  - 输入电压：4.3~5.4V
  - USB输入电压: 4.75~5.25V
  - 伺服导轨输入电压：0~36V

- 重量和尺寸:
  - 重量：99g
  - 尺寸：44_84_12mm

- 其它特性:
  - 工作温度：-20 ~ 80°C （实测值）

## 购买渠道

Order from [CUAV](https://cuav.taobao.com/index.htm?spm=2013.1.w5002-16371268426.2.411f26d9E18eAz).

## 接口定义

![CUAV v5](../../assets/flight_controller/cuav_v5/pixhack_v5_connector.jpg)

:::warning
The RCIN interface is limited to powering the rc receiver and cannot be connected to any power/load.
:::

## 额定电压

_CUAV v5_ can be triple-redundant on the power supply if three power sources are supplied. The three power rails are: **POWER1**, **POWER2** and **USB**.

:::info
The output power rails **FMU PWM OUT** and **I/O PWM OUT** (0V to 36V) do not power the flight controller board (and are not powered by it).
You must supply power to one of **POWER1**, **POWER2** or **USB** or the board will be unpowered.
:::

**Normal Operation Maximum Ratings**

Under these conditions all power sources will be used in this order to power the system:

1. **POWER1** and **POWER2** inputs (4.3V to 5.4V)
2. **USB** input (4.75V to 5.25V)

## 编译固件

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v5_default
```

## 调试接口

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) operate on the **FMU Debug** port.
Simply connect the FTDI cable to the Debug & F7 SWD connector.
To access the I/O Debug port, the user must remove the CUAV v5 shell.
Both ports have standard serial pins and can be connected to a standard FTDI cable (3.3V, but 5V tolerant).

The pinout is as shown.

![CUAV v5 debug](../../assets/flight_controller/cuav_v5/pixhack_v5_debug.jpg)

| 引脚 | CUAV v5 debug                 |
| -- | ----------------------------- |
| 1  | GND                           |
| 2  | FMU-swclk                     |
| 3  | FMU-SWDIO                     |
| 4  | UART7_RX |
| 5  | UART7_TX |
| 6  | VCC                           |

## 串口映射

| UART   | 设备         | Port                                                       |
| ------ | ---------- | ---------------------------------------------------------- |
| UART1  | /dev/ttyS0 | GPS                                                        |
| USART2 | /dev/ttyS1 | TELEM1 (flow control)                   |
| USART3 | /dev/ttyS2 | TELEM2 (flow control)                   |
| UART4  | /dev/ttyS3 | TELEM4                                                     |
| USART6 | /dev/ttyS4 | TX is RC input from SBUS_RC connector |
| UART7  | /dev/ttyS5 | Debug Console                                              |
| UART8  | /dev/ttyS6 | PX4IO                                                      |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

## 外部设备

- [Digital Airspeed Sensor](https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-16371268452.37.6d9f48afsFgGZI&id=9512463037)
- [Telemetry Radio Modules](https://cuav.taobao.com/category-158480951.htm?spm=2013.1.w5002-16371268426.4.410b7a821qYbBq&search=y&catName=%CA%FD%B4%AB%B5%E7%CC%A8)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## 支持的平台/机身

Any multicopter / airplane / rover or boat that can be controlled with normal RC servos or Futaba S-Bus servos.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## 更多信息

- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165).
- [CUAV v5 docs](http://doc.cuav.net/flight-controller/v5-autopilot/en/v5.html)
- [CUAV Github](https://github.com/cuav)
