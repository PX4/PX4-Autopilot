# Holybro Pixhawk 5X

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

_Pixhawk 5X_<sup>&reg;</sup> is the latest update to the successful family of Pixhawk® flight controllers designed and made in collaboration with Holybro<sup>&reg;</sup> and the PX4 team.

It is based on the [Pixhawk​​® Autopilot FMUv5X Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-011%20Pixhawk%20Autopilot%20v5X%20Standard.pdf), [Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf), and [Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).
It comes with the latest PX4 Autopilot​® pre-installed, triple redundancy, temperature-controlled, isolated sensor domain, delivering incredible performance and reliability.

<img src="../../assets/flight_controller/pixhawk5x/pixhawk5x_hero_upright.jpg" width="250px" title="Pixhawk5x Upright Image" /> <img src="../../assets/flight_controller/pixhawk5x/pixhawk5x_exploded_diagram.jpg" width="450px" title="Pixhawk5x Exploded Image" />

:::tip
This autopilot is [supported](../flight_controller/autopilot_pixhawk_standard.md) by the PX4 maintenance and test teams.
:::

## 소개

Inside the Pixhawk® 5X, you can find an STMicroelectronics® based STM32F7, paired with sensor technology from Bosch®, InvenSense®, giving you flexibility and reliability for controlling any autonomous vehicle, suitable for both academic and commercial applications.
The Pixhawk® 5X’s F7 microcontroller has 2MB flash memory and 512KB RAM.
The PX4 Autopilot takes advantage of the increased power and RAM.
Thanks to the updated processing power, developers can be more productive and efficient with their development work, allowing for complex algorithms and models.

The FMUv5X open standard includes high-performance, low-noise IMUs on board, designed for better stabilization.
Triple redundant IMU & double redundant barometer on separate buses.
When the PX4 Autopilot detects a sensor failure, the system seamlessly switches to another to maintain flight control reliability.

An independent LDO powers every sensor set with independent power control.
A newly designed vibration isolations to filter out high-frequency vibration and reduce noise to ensure accurate readings, allowing vehicles to reach better overall flight performances.
External sensor bus (SPI5) has two chip select lines and data-ready signals for additional sensors and payload with SPI-interface, and with an integrated Microchip Ethernet PHY (LAN8742AI-CZ-TR), high-speed communication with mission computers via ethernet is now supported.
Two smart battery monitoring ports (SMBus), support for INA226 SMBus Power module.

The Pixhawk® 5X is perfect for developers at corporate research labs, startups, academics (research, professors, students), and commercial application.

## Key Design Points

- Modular flight controller
  - separated IMU, FMU, and Base system connected by a 100-pin & a 50-pin Pixhawk® Autopilot Bus connector, designed for flexible and customizable systems
- Redundancy
  - 3x IMU sensors & 2x Barometer sensors on separate buses, allowing parallel and continuous operation even in the event of a hardware failure
- Triple redundancy domains
  - Completely isolated sensor domains with separate buses and separate power control
- Temperature-controlled IMUs
  - Onboard IMU heating resistors, allowing optimum working temperature of IMUs
- Vibration isolation system
  - Newly designed system to filter out high frequency vibration and reduce noise to ensure accurate readings
- Ethernet interface
  - For high-speed mission computer integration
- Automated sensor calibration eliminating varying signals and temperature
- Two smart batteries monitoring on SMBus
- Additional GPIO line and 5V for the external NFC reader
- Secure element for secure authentication of the drone (SE050)

## Technical specification

- FMU Processor: STM32F765
  - 32 비트 Arm® Cortex®-M7, 216MHz, 2MB 메모리, 512KB RAM

- IO 프로세서: STM32F100
  - 32 비트 Arm® Cortex®-M3, 24MHz, 8KB SRAM

- On-board Sensors:

  - Accel/Gyro: ICM-20649
  - Accel/Gyro: ICM-42688P
  - 가속도/자이로: ICM-20602
  - 자력계: BMM150
  - Barometer: 2x BMP388

- 인터페이스

  - 16- PWM servo outputs
  - R/C input for Spektrum / DSM
  - Dedicated R/C input for PPM and S.Bus input
  - Dedicated analog / PWM RSSI input and S.Bus output
  - 범용 시리얼 포트 4개
    - 전체 흐름 제어 3개
    - 1.5A 전류 제한이 있는 1 개
    - 1 with I2C and additional GPIO line for external NFC reader
  - 2 GPS ports
    - 1 full GPS & Safety Switch Port
    - 1 basic GPS port
  - 1 I2C port
  - 1 Ethernet port
    - Transformerless Applications
  - 100Mbps
  - 1 SPI bus
  - 2 chip select lines
  - 2 data-ready lines
  - 1 SPI SYNC line
  - 1 SPI reset line
  - 2 CAN Buses for CAN peripheral
    - CAN Bus has individual silent controls or ESC RX-MUX control
  - 2 Power input ports with SMBus
  - 1 AD & IO port
    - 2개의 추가 아날로그 입력
    - 1 PWM/Capture input
    - 2 Dedicated debug and GPIO lines

- 정격 전압

  - 최대 입력 전압: 6V
  - USB 전원 입력: 4.75~5.25V
  - 서보 레일 입력: 0~36V

- 크기

  - Flight Controller Module: 38.8 x 31.8 x 14.6mm
  - Standard Baseboard: 52.4 x 103.4 x 16.7mm

- 중량

  - Flight Controller Module: 23g
  - Standard Baseboard: 51g

- 기타 특성:
  - Operating & storage temperature: -40 ~ 85°c

## 구매처

Order from [Holybro](https://holybro.com/products/pixhawk-5x).

## 조립 및 설정

The [Pixhawk 5X Wiring Quick Start](../assembly/quick_start_pixhawk5x.md) provides instructions on how to assemble required/important peripherals including GPS, Power Module etc.

## 연결

![Pixhawk 5x Wiring Overview](../../assets/flight_controller/pixhawk5x/pixhawk5x_wiring_diagram.jpg)

## 핀배열

![Pixhawk 5X Pinout](../../assets/flight_controller/pixhawk5x/pixhawk5x_pinout.png)

:::info
Connector pin assignments are left to right (i.e. Pin 1 is the left-most pin).
::: infos:

- The [camera capture pin](../camera/fc_connected_camera.md#camera-capture-configuration) (`PI0`) is pin 2 on the AD&IO port, marked above as `FMU_CAP1`.
- _Pixhawk 5X_ pinouts can be downloaded in PDF from from [here](https://github.com/PX4/PX4-user_guide/blob/main/assets/flight_controller/pixhawk5x/pixhawk5x_pinout.pdf) or [here](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Holybro_Pixhawk5X_Pinout.pdf).

## 시리얼 포트 매핑

| UART   | 장치         | 포트                              |
| ------ | ---------- | ------------------------------- |
| USART1 | /dev/ttyS0 | GPS                             |
| USART2 | /dev/ttyS1 | TELEM3                          |
| USART3 | /dev/ttyS2 | 디버그 콘솔                          |
| UART4  | /dev/ttyS3 | UART4 & I2C |
| UART5  | /dev/ttyS4 | TELEM2                          |
| USART6 | /dev/ttyS5 | PX4IO/RC                        |
| UART7  | /dev/ttyS6 | TELEM1                          |
| UART8  | /dev/ttyS7 | GPS2                            |

## 크기

![Pixhawk 5X Dimensions](../../assets/flight_controller/pixhawk5x/pixhawk5x_dimensions_all.jpg)

## 정격 전압

_Pixhawk 5X_ can be triple-redundant on the power supply if three power sources are supplied. The three power rails are: **POWER1**, **POWER2** and **USB**.
The **POWER1** & **POWER2** ports on the Pixhawk 5X uses the 6 circuit [2.00mm Pitch CLIK-Mate Wire-to-Board PCB Receptacle](https://www.molex.com/molex/products/part-detail/pcb_receptacles/5024430670).

**Normal Operation Maximum Ratings**

이러한 조건에서 전원은 아래의 순서대로 시스템에 전원을 공급하여야합니다.

1. **POWER1** and **POWER2** inputs (4.9V to 5.5V)
2. **USB** input (4.75V to 5.25V)

**Absolute Maximum Ratings**

아래의 조건에서 시스템은 전원을 사용하지 않지만(작동하지 않음), 그대로 유지됩니다.

1. **POWER1** and **POWER2** inputs (operational range 4.1V to 5.7V, 0V to 10V undamaged)
2. **USB** input (operational range 4.1V to 5.7V, 0V to 6V undamaged)
3. Servo input: VDD_SERVO pin of **FMU PWM OUT** and **I/O PWM OUT** (0V to 42V undamaged)

**Voltage monitoring**

Digital I2C battery monitoring is enabled by default (see [Quickstart > Power](../assembly/quick_start_pixhawk5x.md#power)).

::: info
Analog battery monitoring via an ADC is not supported on this particular board, but may be supported in variations of this flight controller with a different baseboard.
:::

## 펌웨어 빌드

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v5x_default
```

<a id="debug_port"></a>

## 디버그 포트

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) run on the **FMU Debug** port.

The pinouts and connector comply with the [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full) interface defined in the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) interface (JST SM10B connector).

| 핀                           | 신호                                  | 전압                    |
| --------------------------- | ----------------------------------- | --------------------- |
| 1(red)   | `Vtref`                             | +3.3V |
| 2 (흑)    | Console TX (OUT) | +3.3V |
| 3 (흑)    | Console RX (IN)  | +3.3V |
| 4 (흑)    | `SWDIO`                             | +3.3V |
| 5 (흑)    | `SWCLK`                             | +3.3V |
| 6 (흑)    | `SWO`                               | +3.3V |
| 7 (흑)    | NFC GPIO                            | +3.3V |
| 8 (blk)  | PH11                                | +3.3V |
| 9 (blk)  | nRST                                | +3.3V |
| 10 (blk) | `GND`                               | GND                   |

For information about using this port see:

- [SWD Debug Port](../debug/swd_debug.md)
- [PX4 System Console](../debug/system_console.md) (Note, the FMU console maps to USART3).

## 주변 장치

- [Digital Airspeed Sensor](https://holybro.com/products/digital-air-speed-sensor)
- [Telemetry Radio Modules](https://holybro.com/collections/telemetry-radios?orderby=date)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## 지원 플랫폼 및 기체

일반 RC 서보 또는 Futaba S-Bus 서보로 제어 가능한 모든 멀티콥터/비행기/로버 또는 보트.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## 추가 정보

- [Pixhawk 5X Wiring QuickStart](../assembly/quick_start_pixhawk5x.md)
- [Pixhawk 5X Overview & Specification](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Holybro_Pixhawk5X_Spec_Overview.pdf) (Holybro)
- [Pixhawk 5X Pinouts](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Holybro_Pixhawk5X_Pinout.pdf) (Holybro)
- [FMUv5X reference design pinout](https://docs.google.com/spreadsheets/d/1Su7u8PHp-Y1AlLGVuH_I8ewkEEXt_bHHYBHglRuVH7E/edit#gid=562580340).
- [Pixhawk Autopilot FMUv5X Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-011%20Pixhawk%20Autopilot%20v5X%20Standard.pdf).
- [Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf).
- [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).
