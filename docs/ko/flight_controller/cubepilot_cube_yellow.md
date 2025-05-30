# CubePilot Cube Yellow Flight Controller

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://cubepilot.org/#/home) for hardware support or compliance issues.
:::

The Cube Yellow flight controller is a flexible autopilot intended primarily for manufacturers of commercial systems.

![Cube Yellow](../../assets/flight_controller/cube/yellow/cube_yellow_hero.jpg)

배선을 줄이고 신뢰성을 높이며 조립을 쉽게하기 위해 도메인별 캐리어 보드와 함께 사용하도록 설계되었습니다.
예를 들어, 상용 검사 기체 캐리어보드에는 보조 컴퓨터용 연결이 포함될 수 있는 반면, 레이서 용 캐리어보드는 기체 프레임을 형성하는 ESC를 포함할 수 있습니다.

Cube에는 2 개의 IMU에 진동 차단이 포함되어 있으며, 세 번째 고정 IMU는 참조 백업용으로 사용됩니다.

:::tip
The manufacturer [Cube Docs](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview) contain detailed information, including an overview of the [Differences between Cube Colours](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview#differences-between-cube-colours).
:::

## 주요 특징

- 32bit STM32F777VI (32bit [ARM Cortex M7](https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M7), 400 MHz, Flash 2MB, RAM 512 KB).
- 32 bit STM32F103 failsafe co-processor <!-- check -->
- 14 개 PWM/서보 출력(페일세이프 및 수동 오버라이드 포함 8 개, 보조, 고전력 호환 6 개)
- 추가 주변 장치(UART, I2C, CAN) 다양한 연결 옵션
- 전용 프로세서와 독립형 전원 공급 장치(고정익 적용)로 비행중 복구 기능과 수동 오버라이드 통합 백업 시스템
- Backup system integrates mixing, providing consistent autopilot and manual override mixing modes (fixed-wing use)
- 중복 전원공급장치 및 자동 장애 조치
- 외부 안전 스위치
- 다색 LED 주시각 표시기
- 고전력 멀티톤 피에조 오디오 표시기
- 장기간 고속 로깅용 microSD 카드

<a id="stores"></a>

## 구매처

- [Reseller list](https://www.cubepilot.com/#/reseller/list)

## 조립

[Cube Wiring Quickstart](../assembly/quick_start_cube.md)

## 사양

- **Processor:**
  - STM32F777VI (32bit [ARM Cortex M7](https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M7))
  - 400 MHz
  - 512 KB MB RAM
  - 2 MB Flash
- **Failsafe co-processor:** <!-- inconsistent info on failsafe processor: 32 bit STM32F103 failsafe co-processor http://www.proficnc.com/all-products/191-pixhawk2-suite.html -->
  - STM32F100 (32bit _ARM Cortex-M3_)
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
    - 전압: 4V - 5.7V
    - 정격 전류: 250 mA
  - **POWER:**
    - 입력 전압: 4.1V - 5.7V
    - 정격 입력 전류: 2.5A
    - 정격 입/출력 전력: 14W
- **Dimensions:**
  - **Cube:** 38.25mm x 38.25mm x 22.3mm
  - **Carrier:** 94.5mm x 44.3mm x 17.3mm
- **Interfaces**
  - IO 포트: 14 개의 PWM 서보 출력(IO에서 8 개, FMU에서 6 개)
  - UART (직렬 포트) 5개, 1 개의 고전력 지원, 2x (HW 흐름 제어 포함)
  - CAN 2개(하나는 내부 3.3V 트랜시버, 하나는 확장 커넥터에 있음)
  - **R/C inputs:**
    - Spektrum DSM/DSM2/DSM-X® Satellite 호환 입력
    - Futaba S.BUS® 호환 입력 및 출력
    - PPM 합계 신호 입력
  - RSSI(PWM 또는 전압) 입력
  - I2C
  - SPI
  - 3.3v ADC 입력
  - 내부 microUSB 포트 및 외부 microUSB 포트 확장

## 핀배열과 회로도

Board schematics and other documentation can be found here: [The Cube Project](https://github.com/proficnc/The-Cube).

## 포트

### 위쪽(GPS, TELEM 등)

![Cube Ports - Top (GPS, TELEM etc) and Main/AUX](../../assets/flight_controller/cube/cube_ports_top_main.jpg)

## 시리얼 포트 매핑

| UART   | 장치         | 포트                                |
| ------ | ---------- | --------------------------------- |
| USART2 | /dev/ttyS0 | TELEM1 (흐름 제어) |
| USART3 | /dev/ttyS1 | TELEM2 (흐름 제어) |
| UART4  | /dev/ttyS2 | GPS1                              |
| USART6 | /dev/ttyS3 | PX4IO                             |
| UART7  | /dev/ttyS4 | CONSOLE/ADSB-IN                   |
| UART8  | /dev/ttyS5 | GPS2                              |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/boards/hex/cube-orange/default.px4board -->

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/boards/hex/cube-orange/nuttx-config/nsh/defconfig#L194-L200 -->

### 디버그 포트

![Cube Debug Ports](../../assets/flight_controller/cube/cube_ports_debug.jpg)

### USB/SDCard 포트

![Cube USB/SDCard Ports](../../assets/flight_controller/cube/cube_ports_usb_sdcard.jpg)

## 펌웨어 빌드

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make cubepilot_cubeyellow
```

## 문제점

Cube의 CAN1과 CAN2의 실크 스크린이 뒤집힙니다. CAN1은 CAN2이고 그 반대의 경우도 마찬가지입니다.

## 추가 정보 및 문서

- [Cube Wiring Quickstart](../assembly/quick_start_cube.md)
- Cube 문서 (제조사) :
  - [Cube Module Overview](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview)
  - [Cube User Manual](https://docs.cubepilot.org/user-guides/autopilot/the-cube-user-manual)
  - [Mini Carrier Board](https://docs.cubepilot.org/user-guides/carrier-boards/mini-carrier-board)
