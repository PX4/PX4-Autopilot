# CUAV Pixhawk V6X

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

_Pixhawk V6X_<sup>&reg;</sup> is the latest update to the successful family of Pixhawk® flight controllers designed and made in collaboration with CUAV<sup>&reg;</sup> and the PX4 team.

It is based on the [Pixhawk​​® Autopilot FMUv6X Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-012%20Pixhawk%20Autopilot%20v6X%20Standard.pdf), [Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf), and [Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).

![Pixhawk V6X](../../assets/flight_controller/cuav_pixhawk_v6x/pixhawk_v6x.jpg)

:::tip
This autopilot is [supported](../flight_controller/autopilot_pixhawk_standard.md) by the PX4 maintenance and test teams.
:::

Pixhawk<sup>&reg;</sup> V6X brings you the ultimate in performance, stability and reliability in all aspects.

- Arm® Cortex®-M7 processor (STM32H753) with Floating Point Unit (FPU), 480MHz high-speed operations and 2MB flash.
  Developers can be more productive and efficient, allowing for more complex algorithms and models.
- High-performance on-board, low-noise IMU and automotive-grade magnetic compass based on FMUv6X open standard.
  It aims to achieve better stability and anti-interference ability.
- Triple redundant IMU & double redundant barometer on separate buses.
  When the PX4 Autopilot detects a sensor failure, the system seamlessly switches to another to maintain flight control reliability.
- An independent LDO powers every sensor set with independent power control.
  A vibration isolation System to filter out high-frequency vibration and reduce noise to ensure accurate readings, allowing vehicles to reach better overall flight performances.
- External sensor bus (SPI5) has two chip select lines and data-ready signals for additional sensors and payload with SPI-interface.
- Integrated Microchip Ethernet PHY for high-speed communication over Ethernet with onboard devices such as mission computers.
- Newly designed vibration isolation system to filter out high frequency vibration and reduce noise to ensure accurate readings.
- IMUs are temperature-controlled by onboard heating resistors, allowing optimum working temperature of IMUs&#x20;
- Modular flight controller: separated IMU, FMU, and Base system connected by a 100-pin & a 50-pin Pixhawk®​ Autopilot Bus connector.

The Pixhawk® V6X is ideal for corporate research labs, academic research and commercial applications.

### Processors & Sensors

- FMU Processor: STM32H753
  - 32 Bit Arm® Cortex®-M7, 480MHz, 2MB flash memory, 1MB RAM
- IO Processor: STM32F103
  - 32 Bit Arm® Cortex®-M3, 72MHz, 20KB SRAM
- 내장 센서 :
  - Accel/Gyro: BMI088
  - Accel/Gyro: ICM-42688-P
  - Accel/Gyro: ICM-20649
  - Mag: RM3100
  - Barometer: 2x ICP-20100

### Electrical data

- Voltage Ratings:
  - Max input voltage: 5.7V
  - USB Power Input: 4.75\~5.25V
  - Servo Rail Input: 0\~9.9V
- Current Ratings:
  - TELEM1 and GPS2 combined output current limiter: 1.5A
  - All other port combined output current limiter: 1.5A

### 인터페이스

- 16- PWM servo outputs
- 1 Dedicated R/C input for Spektrum / DSM and S.Bus with analog / PWM RSSI input
- 3 TELEM Ports（with full flow control）
- 1 UART4(Seial and I2C)
- 2 GPS ports
  - 1 full GPS plus Safety Switch Port(GPS1)
  - 1 basic GPS port(with I2C,GPS2)
- 2 USB Ports
  - 1 TYPE-C
  - JST GH1.25
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
- 4 power input ports
  - 2 Dronecan/UAVCAN power inputs
  - 2 SMBUS/I2C power inputs
- 1 AD & IO port
  - 2 additional analog input(3.3 and 6.6v）
  - 1 PWM/Capture input
- 2 Dedicated debug
  - FMU debug
  - IO 디버그

### Mechanical data

- 중량
  - Flight Controller Module: 99g
  - Core module: 43g
  - Baseboard: 56g
- Operating & storage temperature: -20 ~ 85°c
- Size

  - Flight controller

    ![Pixhawk V6X](../../assets/flight_controller/cuav_pixhawk_v6x/v6x_size.jpg)

  - Core module

    ![Pixhawk V6X](../../assets/flight_controller/cuav_pixhawk_v6x/core.png)

## 구매처

Order from [CUAV](https://store.cuav.net/).

## 조립 및 설정

The [Pixhawk V6X Wiring Quick Start](../assembly/quick_start_cuav_pixhawk_v6x.md) provides instructions on how to assemble required/important peripherals including GPS, Power Module etc.

## 핀배열

![Pixhawk V6x Pinout](../../assets/flight_controller/cuav_pixhawk_v6x/pixhawk_v6x_pinouts.png)

참고:

- The [camera capture pin](../camera/fc_connected_camera.md#camera-capture-configuration) (`PI0`) is pin 2 on the AD&IO port, marked above as `FMU_CAP1`.

## 시리얼 포트 매핑

| UART   | 장치         | 포트       |
| ------ | ---------- | -------- |
| USART1 | /dev/ttyS0 | GPS      |
| USART2 | /dev/ttyS1 | TELEM3   |
| USART3 | /dev/ttyS2 | 디버그 콘솔   |
| UART4  | /dev/ttyS3 | UART4    |
| UART5  | /dev/ttyS4 | TELEM2   |
| USART6 | /dev/ttyS5 | PX4IO/RC |
| UART7  | /dev/ttyS6 | TELEM1   |
| UART8  | /dev/ttyS7 | GPS2     |

## 정격 전압

_Pixhawk V6X_ can be triple-redundant on the power supply if three power sources are supplied.
The three power rails are: **POWERC1/POWER1**, **POWERC2/POWER2** and **USB**.

- **POWER C1** and **POWER C2** are DroneCAN/UAVCAN battery interfaces (recommended)；**POWER1** and **POWER2** are SMbus/I2C battery interfaces (backup).
- **POWER C1** and **POWER1** use the same power switch, **POWER C2** and **POWER2** use the same power switch.

**Normal Operation Maximum Ratings**

이러한 조건에서 전원은 아래의 순서대로 시스템에 전원을 공급하여야합니다.

1. **POWER C1**, **POWER C2**, **POWER1** and **POWER2** inputs (4.75V to 5.7V)
2. **USB** input (4.75V to 5.25V)

**Absolute Maximum Ratings**

아래의 조건에서 시스템은 전원을 사용하지 않지만(작동하지 않음), 그대로 유지됩니다.

1. **POWER1** and **POWER2** inputs (operational range 4.7V to 5.7V, 0V to 10V undamaged)
2. **USB input** (operational range 4.7V to 5.7V, 0V to 6V undamaged)
3. **Servo input:** `VDD_SERVO` pin of **FMU PWM OUT** and **I/O PWM OUT** (0V to 42V undamaged)

**Voltage monitoring**

Digital DroneCAN/UAVCAN battery monitoring is enabled by default (see [Quickstart > Power](../assembly/quick_start_cuav_pixhawk_v6x.md#power)).

:::info
Analog battery monitoring via an ADC is not supported on this particular board, but may be supported in variations of this flight controller with a different baseboard.
:::

## 펌웨어 빌드

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v6x_default
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

이 포트의 배선과 사용 정보는 다음을 참조하십시오.

- [PX4 System Console](../debug/system_console.md#pixhawk_debug_port) (Note, the FMU console maps to USART3).
- [SWD Debug Port](../debug/swd_debug.md)

## 주변 장치

- [Digital Airspeed Sensor](https://holybro.com/products/digital-air-speed-sensor)
- [Telemetry Radio Modules](https://holybro.com/collections/telemetry-radios?orderby=date)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## 지원 플랫폼 및 기체

일반 RC 서보 또는 Futaba S-Bus 서보로 제어 가능한 모든 멀티콥터/비행기/로버 또는 보트.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## 추가 정보

- [CUAV Docs](https://doc.cuav.net/) (CUAV)
- [Pixhawk V6X Wiring QuickStart](../assembly/quick_start_cuav_pixhawk_v6x.md)
- [Pixhawk Autopilot FMUv6X Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-012%20Pixhawk%20Autopilot%20v6X%20Standard.pdf)
- [Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)
- [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)
