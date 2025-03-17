# MindPX 하드웨어

:::warning
PX4 does not manufacture this (or any) autopilot.
하드웨어 지원과 호환 문제는 [제조사](http://mindpx.net)에 문의하십시오.
:::

AirMind<sup>&reg;</sup> [MindPX](http://mindpx.net) 시리즈는 Pixhawk<sup>&reg;</sup>에서 파생된 차세대 자동조종장치입니다.

![MindPX 콘트롤러](../../assets/hardware/hardware-mindpx.png)

:::info
이 비행 컨트롤러는 [제조업체에서 지원](../flight_controller/autopilot_manufacturer_supported.md)합니다.
:::

## 요약

:::info
주요 하드웨어 문서는 [여기](http://mindpx.net/assets/accessories/Specification9.18_3_pdf.pdf)를 참고하십시오.
:::

MindPX는 Pixhawk<sup>&reg;</sup>에서 분기된 차세대 자동조종장치로, 회로도와 구조가 수정되었으며, 무인기체를 보다 스마트하고 사용하기 용이하도록 새로운 기능으로 더욱 강화되었습니다.

MindPX는 총 PWM 출력 채널을 16 (8개의 주출력 + 8 aux 출력)으로 증가시킵니다.
MindPX는보다 복잡한 VTOL 구성과보다 정밀한 제어를 지원할 수 있습니다.
MindPX는보다 복잡한 VTOL 구성과보다 정밀한 제어를 지원할 수 있습니다.MindPX는 하나의 단일 FMU에서 메인과 AUX 출력을 구현하여,  FMU-V4 기반 비행 컨트롤러에 특히 유용합니다.

![](../../assets/hardware/hardware-mindpx-specs.png)

- 메인시스템 온칩 : STM32F427

  - CPU : 32 비트, 168 MHz ARM 코어 텍스<sup>&reg;</sup> FPU 포함 M4
  - RAM : 256KB SRAM
  - 2MB 플래시
  - ST 마이크로 LSM303D 14 비트 가속도계/자력계
  - MEAS MS5611 기압계
  - InvenSense<sup>&reg;</sup> MPU6500 통합 6축 센서

- 주요 기능 :
  - CNC 가공 가볍고 견고한 알루미늄 합금 케이스
  - 내장 IMU 이중화 내장
  - 총 16개의 PWM 출력 채널 (8 메인 + 8 보조)
  - 플로우 연결을 위한 여분의 I2C 포트 1 개.
  - 보조 컴퓨터 연결 용 추가 USB 포트 1 개(내장 UART-USB 변환기)
  - 개발용 공개 디버그 포트

## 빠른 시작

### 장착

![MindPX 장착](../../assets/hardware/hardware-mindpx-mounting.png)

### 배선

![MindPX 배선 1](../../assets/hardware/hardware-mindpx-wiring1.png)

![MindPX 배선 2](../../assets/hardware/hardware-mindpx-wiring2.png)

### 핀

![MindPX 핀배열](../../assets/hardware/hardware-mindpx-pin.png)

|  번호 |                  설명                 |  번호 |                             설명                             |
| :-: | :---------------------------------: | :-: | :--------------------------------------------------------: |
|  1  |                  전원                 |  9  |             I2C2 (MindFLow)             |
|  2  | 디버그 (부트로더 새로 고침) |  10 |            USB2 (직렬 2 - USB)            |
|  3  | USB1 (펌웨어 새로 고침) |  11 |                           UART4,5                          |
|  4  |                 재설정                 |  12 | UART1 (텔레메트리)Context \| Request Context |
|  5  |    UART3 (GPS)   |  13 |                             CAN                            |
|  6  |   I2C1 (외부 나침반)  |  14 |                             ADC                            |
|  7  |               TF 카드 슬롯              |  15 |                             삼색등                            |
|  8  |  NRF/SPI(원격 제어)  |  16 |                           Looper                           |

### 라디오 수신기

MindPX는 PPM/SBUS/DSM/DSM2/DSMX를 포함한 다양한 무선 수신기를 V2.6부터 지원합니다.
MindPX는 FrSky<sup>&reg;</sup> 양방향 텔레메트리 D와 S.Port도 지원합니다.

자세한 핀 다이어그램은 [사용 설명서](http://mindpx.net/assets/accessories/UserGuide9.18_2_pdf.pdf)를 참조하십시오

### 펌웨어 빌드

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make airmind_mindpx-v2_default
```

### 보조 컴퓨터 PC 연결

MindPX에는 보드에는 USB-TO-UART 브리지 IC가 있습니다.
마이크로 USB-USB  A형 케이블로 연결합니다.
마이크로 USB 끝을 MindPX의 'OBC'포트에 연결하고, USB 유형 A 끝을 보조 컴퓨터에 연결합니다.

그리고, 최대 BAUD 속도는 px4 제품군과 동일하며 최대 921600입니다.

## 사용자 가이드

:::info
사용자 가이드는 [여기](http://mindpx.net/assets/accessories/UserGuide9.18_2_pdf.pdf)를 참고하십시오.
:::

## 구매처

MindRacer는 인터넷 [AirMind Store](http://drupal.xitronet.com/?q=catalog)에 구매할 수 있습니다.
Amazon<sup>&reg;</sup> 또는 eBay<sup>&reg;</sup>에서도 MindRacer를 구매할 수 있습니다.

## 시리얼 포트 매핑

| UART   | 장치         | 포트     |
| ------ | ---------- | ------ |
| USART1 | /dev/ttyS0 | RC     |
| USART2 | /dev/ttyS1 | TELEM1 |
| USART3 | /dev/ttyS2 | TELEM2 |
| UART4  | /dev/ttyS3 | GPS1   |
| USART6 | /dev/ttyS4 | ?      |
| UART7  | /dev/ttyS5 | 디버그 콘솔 |
| UART8  | /dev/ttyS6 | ?      |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

## 지원

자세한 내용은 http://www.mindpx.org를 참고하십시오.
Or you can send email to [support@mindpx.net](mailto:support@mindpx.net) for any inquiries or help.
