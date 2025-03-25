# Holybro Pix32 v6

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

_Pix32 v6_<sup>&reg;</sup> is the latest update to the pix32 v5 flight controllers. It is a variant of the Pixhawk 6C with a modular design and shares the same FMUv6C Target. It is comprised of a separate flight controller and carrier board which are connected by a [100 pin connector](https://docs.holybro.com/autopilot/pix32-v6/download). It is designed for those pilots who need a high power, flexible and customizable flight control system.

It is equipped with a high performance H7 Processor, and comes with IMU redundancy, temperature-controlled IMU board, and cost effective design, delivering incredible performance and reliability. It complies with the [Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).

<img src="../../assets/flight_controller/pix32v6/pix32v6_fc_only.png" width="550px" title="pix32v6 Upright Image" />

<!--
:::tip
This autopilot is [supported](../flight_controller/autopilot_pixhawk_standard.md) by the PX4 maintenance and test teams.
:::
-->

## 소개

Inside the Pix32 v6, you can find an STMicroelectronics® based STM32H743, paired with sensor technology from Bosch® & InvenSense®, giving you flexibility and reliability for controlling any autonomous vehicle, suitable for both academic and commercial applications.

The Pix32 v6’s H7 MCU contain the Arm® Cortex®-M7 core running up to 480 MHz, has 2MB flash memory and 1MB RAM. Thanks to the updated processing power, developers can be more productive and efficient with their development work, allowing for complex algorithms and models. It includes high-performance, low-noise IMUs on board, designed to be cost effective while having IMU redundancy. A vibration isolation System to filter out high-frequency vibration and reduce noise to ensure accurate readings, allowing vehicles to reach better overall flight performances.

This flight controller is perfect for people that is looking for a affordable and modular flight controller that can use a customized baseboard. We have made the [pix32 v6 base board schematics public](https://docs.holybro.com/autopilot/pix32-v6/download), you can either make a custom carrier board yourself or just let us help you with it. By using a customize baseboard, you can make sure that the physical size, pinouts and power distribution requirements match your vehicle perfectly, ensuring that you have all the connections you need and none of the expense and bulk of connectors you don’t.

**Key Design Points**

- High performance STM32H743 Processor with more computing power & RAM
- New cost-effective design with low-profile form factor
- Integrated vibration isolation system to filter out high frequency vibration and reduce noise to ensure accurate readings
- IMUs are temperature-controlled by onboard heating resistors, allowing optimum working temperature of IMUs

# Technical Specification

### **Processors & Sensors**

- FMU Processor: STM32H743&#x20;
  - 32 Bit Arm® Cortex®-M7, 480MHz, 2MB memory, 1MB SRAM&#x20;
- IO Processor: STM32F103
  - &#x20;32 Bit Arm® Cortex®-M3, 72MHz, 64KB SRAM&#x20;
- On-board sensors&#x20;
  - &#x20;Accel/Gyro: ICM-42688-P&#x20;
  - Accel/Gyro: BMI055&#x20;
  - Mag: IST8310&#x20;
  - 기압계: MS5611

### **Electrical data**

- Voltage Ratings:
  - 최대 입력 전압: 6V
  - USB Power Input: 4.75\~5.25V
  - Servo Rail Input: 0\~36V
- Current Ratings:
  - TELEM1 Max output current limiter: 1.5A
  - All other port combined output current limiter: 1.5A

### **Mechanical data**

- FC Module Dimensions: 44.8 x 44.8 x 13.5
- FC Module Weight: 36g

### **Interfaces**

- 16- PWM servo outputs (8 from IO, 8 from FMU)

- 범용 시리얼 포트 3개
  - `TELEM1` - Full flow control, separate 1.5A current limit
  - `TELEM2` - Full flow control
  - `TELEM3`

- 2 GPS ports
  - `GPS1` - Full GPS port (GPS plus safety switch)
  - `GPS2` - Basic GPS port

- 1 I2C port
  - Supports dedicated I2C calibration EEPROM located on sensor module

- 2개의 CAN 버스
  - CAN Bus has individual silent controls or ESC RX-MUX control

- 2 Debug ports:
  - FMU Debug
  - I/O Debug

- Dedicated R/C input for Spektrum / DSM and S.BUS, CPPM, analog / PWM RSSI

- Dedicated S.BUS output

- 2 Power input ports (Analog)

- 기타 특성:
  - Operating & storage temperature: -40 ~ 85°c

## 구매처

Order from [Holybro](https://holybro.com/collections/autopilot-flight-controllers/products/pix32-v6).

## 핀배열

- [Holybro Pix32 v6 Baseboard Ports Pinout](https://docs.holybro.com/autopilot/pix32-v6/pix32-v6-baseboard-ports)
- [Holybro Pix32 v6 Baseboard Ports Pinout](https://docs.holybro.com/autopilot/pix32-v6/pix32-v6-mini-base-ports)

## 시리얼 포트 매핑

| UART   | 장치         | 포트     |
| ------ | ---------- | ------ |
| USART1 | /dev/ttyS0 | GPS1   |
| USART2 | /dev/ttyS1 | TELEM3 |
| USART3 | /dev/ttyS2 | 디버그 콘솔 |
| UART5  | /dev/ttyS3 | TELEM2 |
| USART6 | /dev/ttyS4 | PX4IO  |
| UART7  | /dev/ttyS5 | TELEM1 |
| UART8  | /dev/ttyS6 | GPS2   |

## 크기

- [Pix32v6 Dimensions](https://docs.holybro.com/autopilot/pix32-v6/dimensions)

## 정격 전압

_Pix32 v6_ can be triple-redundant on the power supply if three power sources are supplied. The three power rails are: **USB**, **POWER1**, **POWER2** (N/A on Pix32 v6 Mini-Baseboard) .

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

Pix32 v6 uses analog power modules.

Holybro makes various analog [power modules](../power_module/index.md) for different need.

- [PM02 Power Module](../power_module/holybro_pm02.md)
- [PM06 Power Module](../power_module/holybro_pm06_pixhawk4mini_power_module.md)
- [PM07 Power Module](../power_module/holybro_pm07_pixhawk4_power_module.md)

## 펌웨어 빌드

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v6c_default
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

- [Holybro Docs](https://docs.holybro.com/) (Holybro)
- [Reference: Pixhawk 6C Wiring QuickStart](../assembly/quick_start_pixhawk6c.md)
- [PM02 Power Module](../power_module/holybro_pm02.md)
- [PM06 Power Module](../power_module/holybro_pm06_pixhawk4mini_power_module.md)
- [PM07 Power Module](../power_module/holybro_pm07_pixhawk4_power_module.md)
- FMUv6C reference design pinout.
- [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).
