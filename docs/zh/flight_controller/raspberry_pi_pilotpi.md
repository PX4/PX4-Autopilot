# 树莓派 PilotPi 拓展板

<LinkedBadge type="warning" text="Experimental" url="../flight_controller/autopilot_experimental.html"/>

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](mailto:lhf2613@gmail.com) for hardware support or compliance issues.
:::

The _PilotPi_ shield is a fully functional solution to run PX4 autopilot directly on Raspberry Pi.
It is designed to be a low-cost but highly scalability platform with continuous updates from both Linux and PX4 sides.
No proprietary driver is required, as all components have upstream support from RPi and PX4 community.
PCB and schematic are open source as well.

![PilotPi with RPi 4B](../../assets/flight_controller/pilotpi/hardware-pilotpi4b.png)

## 总览

- 支持的树莓派：
  - 树莓派 2B/3B/3B+/4B
- 支持的操作系统：
  - Raspberry Pi OS
  - Ubuntu Server (armhf/arm64)
- 加速度计/角速度计：
  - ICM42688P
- 磁力计：
  - IST8310
- 气压计：
  - MS5611
- PWM：
  - PCA9685
- ADC：
  - ADS1115
- 电源：
  - 3~6S 电池 具有内置电压监测
  - 通过USB线启动树莓派
- Availability: _preparing for shipping_

## 连接

Shield provides:

- 16 x PWM 输出通道
- GPS 连接器
- 数传连接器
- External I2C bus connector (**Note:** conflicts with CSI camera)
- 遥控输入口（SBUS 协议）
- 3 x 0~5V ADC 通道
- 2\*8 2.54mm 排插，引出未使用的 GPIO

Direct accessible from RPi:

- 4x USB 连接器
- CSI connector(**Note:** conflict with external I2C bus)
- 其它

## 推荐接线

![PilotPi PowerPart wiring](../../assets/flight_controller/pilotpi/pilotpi_pwr_wiring.png)

![PilotPi SensorPart wiring](../../assets/flight_controller/pilotpi/pilotpi_sens_wiring.png)

## 针脚定义

:::warning
It still uses old GH1.25 connectors.
Wiring is compatible with Pixhawk 2.4.8
:::

### 连接器

#### GPS 连接器

Mapped to `/dev/ttySC0`

| 针脚 | 信号  | 电压   |
| -- | --- | ---- |
| 1  | VCC | +5V  |
| 2  | TX  | +3v3 |
| 3  | RX  | +3v3 |
| 4  | NC  | +3v3 |
| 5  | NC  | +3v3 |
| 6  | GND | GND  |

#### 数传连接器

Mapped to `/dev/ttySC1`

| 针脚 | 信号                   | 电压   |
| -- | -------------------- | ---- |
| 1  | VCC                  | +5V  |
| 2  | TX                   | +3v3 |
| 3  | RX                   | +3v3 |
| 4  | CTS                  | +3v3 |
| 5  | RTS: | +3v3 |
| 6  | GND                  | GND  |

#### 外部 I2C 总线连接器

Mapped to `/dev/i2c-0`

| 针脚 | 信号  | 电压                          |
| -- | --- | --------------------------- |
| 1  | VCC | +5V                         |
| 2  | SCL | +3v3(上拉) |
| 3  | SDA | +3v3(上拉) |
| 4  | GND | GND                         |

#### RC & ADC2/3/4

RC is mapped to `/dev/ttyAMA0` with signal inverter switch on RX line.

| 针脚 | 信号  | 电压                       |
| -- | --- | ------------------------ |
| 1  | RC  | +3V3~+5V |
| 2  | VCC | +5V                      |
| 3  | GND | GND                      |

- ADC1 内部连接到分压电路以监测电池电压。
- ADC2 空闲。
- ADC3 可以连接模拟量空速计。
- ADC4 在 ADC 和 VCC 之间有一个跳线帽，监测系统电压。

| 针脚 | 信号   | 电压                     |
| -- | ---- | ---------------------- |
| 1  | ADCx | 0V~+5V |
| 2  | VCC  | +5V                    |
| 3  | GND  | GND                    |

:::info
ADC3 & 4 have an alternative VCC source
When 'Vref' switch is on, 'VCC' pin is driven by REF5050.
:::

#### 拓展板顶部引出的未使用的GPIO

| 拓展板Pin | BCM号 | WiringPi号 | 树莓派Pin |
| ------ | ---- | --------- | ------ |
| 1      | 3V3  | 3v3       | 3V3    |
| 2      | 5V   | 5V        | 5V     |
| 3      | 4    | 7         | 7      |
| 4      | 14   | 15        | 8      |
| 5      | 17   | 0         | 11     |
| 6      | 27   | 2         | 13     |
| 7      | 22   | 3         | 15     |
| 8      | 23   | 4         | 16     |
| 9      | 7    | 11        | 26     |
| 10     | 5    | 21        | 29     |
| 11     | 6    | 22        | 31     |
| 12     | 12   | 26        | 32     |
| 13     | 13   | 23        | 33     |
| 14     | 16   | 27        | 36     |
| 15     | 26   | 25        | 37     |
| 16     | GND  | GND       | GND    |

### 开关

#### 遥控信号反相器

This switch will decide the signal polarity of RX line: `UART_RX = SW xor RC_INPUT`

- 开启：适合SBUS(反转信号)
- 关闭：保留

#### 参考压

ADC 3 & 4 will have VCC driven by:

- 开启开关时：由REF5050驱动
- 关闭开关时：从树莓派5V取电

#### 启动模式

This switch is connected to Pin22(BCM25).
System rc script will check its value and decide whether PX4 should start alongside with system booting or not.

- 开启：开机自启 PX4
- 关闭：不启动 PX4

## 开发者快速指南

Refer to specific instructions for the OS running on your RPi:

- [Raspberry Pi OS Lite (armhf)](raspberry_pi_pilotpi_rpios.md)
- [Ubuntu Server (arm64 & armhf)](raspberry_pi_pilotpi_ubuntu_server.md)
