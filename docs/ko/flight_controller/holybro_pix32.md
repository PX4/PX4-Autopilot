# Holybro pix32 Flight Controller (Discontinued)

<Badge type="info" text="Discontinued" />

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

The Holybro<sup>&reg;</sup> [pix32 autopilot](https://holybro.com/collections/autopilot-flight-controllers/products/pix32pixhawk-flight-controller) (also known as "Pixhawk 2", and formerly as HKPilot32) is based on the [Pixhawk<sup>&reg;</sup>-project](https://pixhawk.org/) **FMUv2** open hardware design.
This board is based on hardware version Pixhawk 2.4.6.
It runs the PX4 flight stack on the [NuttX](https://nuttx.apache.org/) OS.

![pix32](../../assets/flight_controller/holybro_pix32/pix32_hero.jpg)

As a CC-BY-SA 3.0 licensed Open Hardware design, schematics and design files should be [available here](https://github.com/PX4/Hardware).

:::tip
The Holybro pix32 is software compatible with the [3DR Pixhawk 1](../flight_controller/pixhawk.md).
It is not connector compatible, but is otherwise physically very similar to the 3DR Pixhawk or mRo Pixhawk.
:::

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## 주요 특징

- Main System-on-Chip: [STM32F427](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1789)
  - CPU: FPU가있는 32 비트 STM32F427 코어 텍스<sup>&reg;</sup> M4 코어
  - RAM: 168 MHz/256 KB
  - Flash: 2 MB
- 페일세이프 시스템 온칩 : STM32F103
- 센서:
  - ST Micro L3GD20 3축 16비트 자이로스코프
  - ST Micro LSM303D 3축 14비트 가속도계/자력계
  - Invensense<sup>&reg;</sup> MPU 6000 3축 가속도계/자이로스코프
  - MEAS MS5611 기압계
- 크기/중량
  - 크기: 81x44x15mm
  - 중량: 33.1g
- GPS : 나침반 내장 u-blox<sup>&reg;</sup> 초정밀 Neo-7M
- 입력 전압 : 2 ~ 10s (7.4 ~ 37V)

### 연결성

- I2C 1개
- CAN 2 개
- 3.3 및 6.6V ADC 입력
- UART (직렬 포트) 5개, 1 개의 고전력 지원, 2x (HW 흐름 제어 포함)
- 최대 DX8의 Spektrum DSM/DSM2/DSM-X® Satellite 호환 입력(DX9 이상은 지원되지 않음)
- Futaba<sup>&reg;</sup> S.BUS 호환 입력 및 출력
- PPM 합계 신호
- RSSI(PWM 또는 전압) 입력
- SPI
- 외부 microUSB 포트
- Molex PicoBlade 커넥터

## 구매처

[shop.holybro.com](https://holybro.com/collections/autopilot-flight-controllers/products/pix32pixhawk-flight-controller)

### 소품

- [Digital airspeed sensor](https://holybro.com/products/digital-air-speed-sensor)
- [Hobbyking<sup>&reg;</sup> Wifi Telemetry](https://hobbyking.com/en_us/apm-pixhawk-wireless-wifi-radio-module.html)
- [HolyBro SiK Telemetry Radio (EU 433 MHz, US 915 MHz)](../telemetry/holybro_sik_radio.md)

## 펌웨어 빌드

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v2_default
```

## 디버그 포트

See [3DR Pixhawk 1 > Debug Ports](../flight_controller/pixhawk.md#debug-ports).

## 핀배열과 회로도

The board is based on the [Pixhawk project](https://pixhawk.org/) **FMUv2** open hardware design.

- [FMUv2 + IOv2 schematic](https://raw.githubusercontent.com/PX4/Hardware/master/FMUv2/PX4FMUv2.4.5.pdf) -- Schematic and layout

:::info
As a CC-BY-SA 3.0 licensed Open Hardware design, all schematics and design files are [available](https://github.com/PX4/Hardware).
:::

## 시리얼 포트 매핑

| UART   | 장치         | 포트                                |
| ------ | ---------- | --------------------------------- |
| UART1  | /dev/ttyS0 | IO 디버그                            |
| USART2 | /dev/ttyS1 | TELEM1 (흐름 제어) |
| USART3 | /dev/ttyS2 | TELEM2 (흐름 제어) |
| UART4  |            |                                   |
| UART7  | 콘솔         |                                   |
| UART8  | SERIAL4    |                                   |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->
