# CUAV V5+ 자동조종장치

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

_V5+_<sup>&reg;</sup> is an advanced autopilot manufactured by CUAV<sup>&reg;</sup>.
CUAV<sup>&reg;</sup>와 PX4팀이 공동으로 설계하였습니다.

자동조종장치는 상용시스템 통합에 권장되지만, 학술 연구와 기타 용도에도 적합합니다.

![V5+ AutoPilot - hero image](../../assets/flight_controller/cuav_v5_plus/v5+_01.png)

주요 기능은 다음과 같습니다.

- Full compatibility with the [Pixhawk project](https://pixhawk.org/) **FMUv5** design standard and uses the [Pixhawk Connector Standard](https://pixhawk.org/pixhawk-connector-standard/) for all external interfaces.
- 더 안정적이고 신뢰할 수 있는 센서와 함께 FMU v3보다 고급 프로세서, RAM 및 플래시 메모리.
- PX4와 펌웨어 호환.
- 모듈식 설계를 통해 사용자는 자신의 캐리어 보드를 설정할 수 있습니다.
- 고성능 충격흡수 시스템을 갖춘 내장형 진동감쇠 시스템.
- 비행 안전 향상을 위한 다중 센서와 전원 시스템.

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## 요약

- 메인 FMU 프로세서: STM32F765
  - 32 비트 Arm® Cortex®-M7, 216MHz, 2MB 메모리, 512KB RAM

- IO 프로세서: STM32F100
  - 32 비트 Arm® Cortex®-M3, 24MHz, 8KB SRAM

- 내장 센서 :

  - 가속도계/자이로스코프 : ICM-20689
  - 가속도계/자이로스코프 : BMI055
  - 자력계 : IST8310
  - 기압계: MS5611

- 인터페이스:
  - PWM 출력 8-14개(IO 6개, FMU 8개)
  - FMU의 전용 PWM/캡처 입력 3 개
  - CPPM 전용 RC 입력
  - 아날로그/PWM RSSI 입력이있는 Spektrum/DSM 및 S.Bus 전용 RC 입력
  - 아날로그/PWM RSSI 입력
  - PWM Servo 출력
  - 범용 시리얼 포트 5개
  - I2C 포트 4개
  - SPI 버스 4개
  - 2 CANBuses with serial ESC
  - 배터리 2 개의 전압 및 전류에 대한 아날로그 입력

- 전원시스템
  - 전원: 4.3~5.4V
  - USB 입력: 4.75~5.25V

- 중량과 크기
  - 중량: 90g
  - Dimensions: 85.5\*42\*33mm

- 기타 특성:

  - 작동 온도: -20 ~ 80°c (측정 값)

## 구매처

<!-- [CUAV Store](https://store.cuav.net/index.php?id_product=95&id_product_attribute=0&rewrite=cuav-new-pixhack-v5-autopilot-m8n-gps-for-fpv-rc-drone-quadcopter-helicopter-flight-simulator-free-shipping-whole-sale&controller=product&id_lang=1) -->

[CUAV Aliexpress](https://www.aliexpress.com/item/32890380056.html?spm=a2g0o.detail.1000060.1.7a7233e7mLTlVl&gps-id=pcDetailBottomMoreThisSeller&scm=1007.13339.90158.0&scm_id=1007.13339.90158.0&scm-url=1007.13339.90158.0&pvid=d899bfab-a7ca-46e1-adf2-72ad1d649822) (International users)

CUAV Taobao (China Mainland users)

:::info
Autopilot may be purchased with included Neo GPS module
:::

<a id="connection"></a>

## 배선

[CUAV V5+ Wiring Quickstart](../assembly/quick_start_cuav_v5_plus.md)

## 핀배열

Download **V5+** pinouts from [here](http://manual.cuav.net/V5-Plus.pdf).

## 정격 전압

_V5+ AutoPilot_ supports redundant power supplies - up to three sources may be used: `Power1`, `Power2` and `USB`.
이러한 소스중 하나 이상에 전원을 공급하여야합니다. 그렇지 않으면, 비행 컨트롤러에 전원이 공급되지 않습니다.

:::info
On FMUv5 based FMUs with PX4IO module (as is the case for the _V5+_), the Servo Power Rail is only monitored by the FMU.
FMU에 의해 전원에 의해 공급받지도 않고, 공급되지도 않습니다.
However, the pins marked **+** are all common, and a BEC may be connected to any of the servo pin sets to power the servo power rail.
:::

**Normal Operation Maximum Ratings**

이러한 조건에서 전원은 아래의 순서대로 시스템에 전원을 공급하여야합니다.

1. `Power1` and `Power2` inputs (4.3V to 5.4V)
2. `USB` input (4.75V to 5.25V)

## 과전류 보호

The _V5+_ has over current protection on the 5 Volt Peripheral and 5 Volt high power, which limits the current to 2.5A.
The _V5+_ has short circuit protection.

:::warning
Up to 2.5 A can be delivered to the connectors listed as pin 1 (although these are only rated at 1 A).
:::

## 펌웨어 빌드

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v5_default
```

## 디버그 포트

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) operate on the **FMU Debug** port (`DSU7`).
보드에는 I/O 디버그 인터페이스가 없습니다.

![Debug port (DSU7)](../../assets/flight_controller/cuav_v5_plus/debug_port_dsu7.jpg)

The debug port (`DSU7`) uses a [JST BM06B](https://www.digikey.com.au/product-detail/en/jst-sales-america-inc/BM06B-GHS-TBT-LF-SN-N/455-1582-1-ND/807850) connector and has the following pinout:

| 핀                         | 신호                              | 전압                    |
| ------------------------- | ------------------------------- | --------------------- |
| 1(red) | 5V+                             | +5V                   |
| 2 (흑)  | DEBUG TX(출력) | +3.3V |
| 3 (흑)  | DEBUG TX(입력) | +3.3V |
| 4 (흑)  | FMU_SWDIO  | +3.3V |
| 5 (흑)  | FMU_SWCLK  | +3.3V |
| 6 (흑)  | GND                             | GND                   |

The product package includes a convenient debug cable that can be connected to the `DSU7` port.
This splits out an FTDI cable for connecting the [PX4 System Console](../debug/system_console.md) to a computer USB port, and SWD pins used for SWD/JTAG debugging.
The provided debug cable does not connect to the SWD port `Vref` pin (1).

![CUAV Debug cable](../../assets/flight_controller/cuav_v5_plus/cuav_v5_debug_cable.jpg)

:::warning
The SWD Vref pin (1) uses 5V as Vref but the CPU is run at 3.3V!

일부 JTAG 어댑터 (SEGGER J-Link)는 Vref 전압을 사용하여 SWD 라인의 전압을 설정합니다.
For direct connection to _Segger Jlink_ we recommended you use the 3.3 Volts from pin 4 of the connector marked `DSM`/`SBUS`/`RSSI` to provide `Vtref` to the JTAG (i.e. providing 3.3V and _NOT_ 5V).

For more information see [Using JTAG for hardware debugging](#using-jtag-for-hardware-debugging).
:::

## 시리얼 포트 매핑

| UART   | 장치         | 포트                                                              |
| ------ | ---------- | --------------------------------------------------------------- |
| UART1  | /dev/ttyS0 | GPS                                                             |
| USART2 | /dev/ttyS1 | TELEM1 (흐름 제어)                               |
| USART3 | /dev/ttyS2 | TELEM2 (흐름 제어)                               |
| UART4  | /dev/ttyS3 | TELEM4                                                          |
| USART6 | /dev/ttyS4 | TX는 SBUS_RC 커넥터의 RC 입력입니다. |
| UART7  | /dev/ttyS5 | 디버그 콘솔                                                          |
| UART8  | /dev/ttyS6 | PX4IO                                                           |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

<a id="optional-hardware"></a>

## 주변 장치

- [Digital Airspeed Sensor](https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-16371268452.37.6d9f48afsFgGZI&id=9512463037)
- [Telemetry Radio Modules](https://cuav.taobao.com/category-158480951.htm?spm=2013.1.w5002-16371268426.4.410b7a821qYbBq&search=y&catName=%CA%FD%B4%AB%B5%E7%CC%A8)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## 지원 플랫폼 및 기체

일반 RC 서보 또는 Futaba S-Bus 서보로 제어 가능한 모든 멀티콥터/비행기/로버 또는 보트.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## 참고

#### 다른 유형의 전원모듈용 커넥터에 디지털 또는 아날로그 전원모듈을 연결하지 마십시오.

아날로그 전원모듈을 디지털 전원모듈 커넥터에 연결하면 해당 버스의 모든 I2C 장치가 중지됩니다.
특히, 경합으로 인하여 GPS의 나침반이 중지되고, 장기적으로 FMU가 손상 될 수도 있습니다.

마찬가지로, 아날로그 커넥터에 연결된 디지털 전원모듈은 작동하지 않으며, 장기적으로 전원 모듈이 손상될 수 있습니다.

## 호환성

CUAV는 몇 가지 차별화된 디자인을 채택하고, 아래에서 설명하는 일부 하드웨어와 호환되지 않습니다.

<a id="compatibility_gps"></a>

#### 다른 장치와 호환되지 않는 GPS

The _Neo v2.0 GPS_ recommended for use with _CUAV V5+_ and _CUAV V5 nano_ is not fully compatible with other Pixhawk flight controllers (specifically, the buzzer part is not compatible and there may be issues with the safety switch).

The UAVCAN [NEO V2 PRO GNSS receiver](http://doc.cuav.net/gps/neo-series-gnss/en/neo-v2-pro.html) can also be used, and is compatible with other flight controllers.

<a id="compatibility_jtag"></a>

#### 하드웨어 디버깅에 JTAG 사용

`DSU7` FMU Debug Pin 1 is 5 volts - not the 3.3 volts of the CPU.

일부 JTAG는이 전압을 사용하여 타겟과 통신시 IO 레벨을 설정합니다.

For direct connection to _Segger Jlink_ we recommended you use the 3.3 Volts of DSM/SBUS/RSSI pin 4 as Pin 1 on the debug connector (`Vtref`).

## 알려진 이슈들

The issues below refer to the _batch number_ in which they first appear.
배치번호는 V01 뒤의 4 자리 생산날짜이며 비행 컨트롤러 측면의 스티커에 표시되어 있습니다.
예를 들어, 일련 번호 Batch V011904((V01은 V5의 번호, 1904는 생산날짜, 즉 배치번호)입니다.

<a id="pin1_unfused"></a>

#### SBUS/DSM/RSSI 인터페이스 Pin1 언퓨즈

:::warning
This is a safety issue.
:::

SBUS/DSM/RSSI 인터페이스에 다른 장비(RC 수신기 제외)를 연결하지 마십시오. 장비가 손상될 수 있습니다.

- _Found:_ Batches V01190904xxxx
- _Fixed:_ Batches later than V01190904xxxx

## 추가 정보

- [CUAV V5+ Manual](http://manual.cuav.net/V5-Plus.pdf)
- [CUAV V5+ docs](http://doc.cuav.net/flight-controller/v5-autopilot/en/v5+.html)
- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165)
- [CUAV Github](https://github.com/cuav)
- [Base board design reference](https://github.com/cuav/hardware/tree/master/V5_Autopilot/V5%2B/V5%2BBASE)
- [CUAV V5+ Wiring Quickstart](../assembly/quick_start_cuav_v5_plus.md)
- [Airframe build-log using CUAV v5+ on a DJI FlameWheel450](../frames_multicopter/dji_f450_cuav_5plus.md)
