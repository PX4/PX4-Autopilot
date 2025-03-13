# Holybro Pixhawk 4 Mini (Discontinued)

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

The _Pixhawk<sup>&reg;</sup> 4 Mini_ autopilot is designed for engineers and hobbyists who are looking to tap into the power of _Pixhawk 4_ but are working with smaller drones.
_Pixhawk 4 Mini_ takes the FMU processor and memory resources from the _Pixhawk 4_ while eliminating interfaces that are normally unused.
This allows the _Pixhawk 4 Mini_ to be small enough to fit in a 250mm racer drone.

_Pixhawk 4 Mini_ was designed and developed in collaboration with Holybro<sup>&reg;</sup> and Auterion<sup>&reg;</sup>.
It is based on the [Pixhawk](https://pixhawk.org/) **FMUv5** design standard and is optimized to run PX4 flight control software.

![Pixhawk4 mini](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_iso_1.png)

:::tip
This autopilot is [supported](../flight_controller/autopilot_pixhawk_standard.md) by the PX4 maintenance and test teams.
:::

## 总览

- 主处理器：STM32F765
  - 32 位 Arm® Cortex®-M7，216MHz，2MB 储存，512KB RAM
- 内置传感器：
  - 加速度计 / 陀螺仪：ICM-20689
  - Accel/Gyro: BMI055 or ICM20602
  - 磁力计：IST8310
  - 气压计：MS5611
- GPS：ublox Neo-M8N GPS/GLONASS 接收器；集成磁力计 IST8310
- 接口：
  - 8 路 PWM 输出
  - FMU 上有 4 路专用 PWM/Capture 输入
  - CPPM专用的RC输入
  - 用于 Spektrum / DSM 与 有模拟 / PWM RSSI 的 S.Bus 的专用遥控输入
  - 3个通用串行口
  - 2 路 I2C 总线
  - 3 路 SPI 总线
  - 1 路 CAN 总线用于 CAN 电调
  - 电池电压 / 电流模拟输入口
  - 2 个模拟输入接口
- 电源系统
  - Power 接口输入：4.75~5.5V
  - USB 电源输入：4.75~5.25V
  - 舵机轨道输入：0~24V
  - 最大电流感应：120A
- 重量和尺寸:
  - Weight: 37.2g
  - Dimensions: 38x55x15.5mm
- 其它特性:
  - 工作温度：-40 ~ 85°C

Additional information can be found in the [_Pixhawk 4 Mini_ Technical Data Sheet](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/pixhawk4mini/pixhawk4mini_technical_data_sheet.pdf).

## 购买渠道

Order from [Holybro](https://holybro.com/collections/autopilot-flight-controllers/products/pixhawk4-mini).

## 接口

![Pixhawk 4 Mini interfaces](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_interfaces.png)

:::warning
The **RC IN** and **PPM** ports are for RC receivers only. These are powered! NEVER connect any servos, power supplies or batteries (or to any connected receiver).
:::

## 针脚定义

Download _Pixhawk 4 Mini_ pinouts from [here](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/pixhawk4mini/pixhawk4mini_pinouts.pdf).

## 尺寸

![Pixhawk 4 Mini Dimensions](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_dimensions.png)

## 额定电压

_Pixhawk 4 Mini_ can have power supply redundancy — if two power sources are supplied. The power rails are: **POWER** and **USB**.

:::info
The output power rail of **MAIN OUT** does not power the flight controller board (and is not powered by it).
You must [supply power](../assembly/quick_start_pixhawk4_mini.md#power) to one of **POWER** or **USB** or the board will be unpowered.
:::

**Normal Operation Maximum Ratings**

Under these conditions all power sources will be used in this order to power the system:

1. **POWER** (4.75V to 5.5V)
2. **USB** input (4.75V to 5.25V)

**Absolute Maximum Ratings**

Under these conditions the system will remain intact.

1. **POWER** input (0V to 6V undamaged)
2. **USB** input (0V to 6V undamaged)
3. Servo input: VDD_SERVO pin of **MAIN OUT** (0V to 24V undamaged)

## 组装 / 设置

The [_Pixhawk 4 Mini_ Wiring Quick Start](../assembly/quick_start_pixhawk4_mini.md) provides instructions on how to assemble required/important peripherals including GPS, Power Management Board, etc.

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

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) run on the **FMU Debug** port.
In order to access these ports, the user must remove the _Pixhawk 4 Mini_ casing.

![Pixhawk 4 Mini FMU Debug](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_fmu_debug.png)

The port has a standard serial pinout and can be connected to a standard FTDI cable (3.3V, but it's 5V tolerant) or a [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation). The pinout uses the standard [Pixhawk debug connector](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) pinout. Please refer to the [wiring](../debug/system_console.md) page for details of how to wire up this port.

## 串口映射

|  UART  |     设备     | QGC Parameter Description |               Port Label on FC              |
| :----: | :--------: | :-----------------------: | :-----------------------------------------: |
|  UART1 | /dev/ttyS0 |            GPS1           |                  GPS Module                 |
| USART2 | /dev/ttyS1 |           TELEM1          |                    TELEM1                   |
| USART3 | /dev/ttyS2 |           TELEM2          |                     N/A                     |
|  UART4 | /dev/ttyS3 |       TELEM/SERIAL4       |                  UART/l2C B                 |
| USART6 | /dev/ttyS4 |            N/A            |                    RC IN                    |
|  UART7 | /dev/ttyS5 |            N/A            |                    Debug                    |
|  UART8 | /dev/ttyS6 |            N/A            | Not connected (no PX4IO) |

## 外部设备

- [Digital Airspeed Sensor](https://holybro.com/products/digital-air-speed-sensor)
- [Telemetry Radio Modules](../telemetry/index.md)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## 支持的平台

Motors and servos are connected to the **MAIN OUT** ports in the order specified for your vehicle in the [Airframe Reference](../airframes/airframe_reference.md).
This reference lists the output port to motor/servo mapping for all supported air and ground frames (if your frame is not listed in the reference then use a "generic" airframe of the correct type).

:::warning
_Pixhawk 4 Mini_ does not have AUX ports.
The board cannot be used with frames that require more than 8 ports or which use AUX ports for motors or control surfaces.
It can be used for airframes that use AUX for non-essential peripherals (e.g. "feed-through of RC AUX1 channel").
:::

## 更多信息

- [_Pixhawk 4 Mini_ Technical Data Sheet](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/pixhawk4mini/pixhawk4mini_technical_data_sheet.pdf)
- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165).
